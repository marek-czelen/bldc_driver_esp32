# Plan napraw: utrata mocy SINUS/FOC pod obciążeniem

## 1. KRYTYCZNE — prawdopodobnie główna przyczyna

### A) Stały błąd kąta obserwera (err=-17) → wektor momentu przesunięty o 64°
- `err[-18..-17]` we WSZYSTKICH dotychczasowych dumpach
- Przy err=-17 entries = 64° el. → moment jest cos(64°) ≈ 0.44 = **tylko 44% nominalnego momentu!**
- To oznacza, że silnik dostaje mniej niż połowę możliwego momentu
- Trzeba sprawdzić PLL_SIGN_FIX — czy po wgraniu nowego FW err spadł

### B) `SINE_SAFE_MAX_DUTY` = 75% ogranicza napięcie
- `SINE_SAFE_MAX_DUTY = PWM_MAX_DUTY * 75 / 100` = 375
- BLOCK daje do 100% (500), SINUS/FOC max 75%
- Pod obciążeniem silnik potrzebuje pełnego napięcia → brakuje 25%
- Do sprawdzenia: zwiększyć do 90-95%

### C) Fałszywe wyzwolenia limitera prądowego (INA180A2 szpilki ADC)
- INA180A2 z low-side shuntem NIE zsynchronizowany z PWM → odczytuje transjenty przełączania
- Komentarz w kodzie wprost mówi "odczyty 20-33A z zasilacza 3A"
- EMA α=0.15 → szpilki 30A wchodzą do filtru jako ~4.5A za każdym razem
- Przy limicie 15A: kilka szpilek pod obciążeniem → `g_maxI_filtered` > 15A → limiter tnie duty
- Hard cutoff 150% × 15A = 22.5A → szpilka 30A → **HARD CUTOFF → 500ms blokada → moc 0!**
- **Sprawdzenie**: wpisać `ilim:0` (wyłączyć limiter) i spróbować znowu

### L) Brak synchronizacji ADC z PWM — źródło fałszywych odczytów prądu
- ADC czytany w `loop()` przez `analogRead()` (~2kHz) — **losowo** trafia w cykl PWM
- ISR MCPWM odpala w dolinie PWM (TEZ, counter=0) — **idealny moment** na odczyt (low-side ON)
- Ale ISR **nie czyta ADC** — zmienne `g_adc_ready_isr` i `g_phase_adc_raw_isr` zadeklarowane, nigdy nie używane
- INA180A2 low-side shunt widzi prąd TYLKO gdy low-side ON (~50% cyklu PWM)
- Losowe trafienie ADC daje: prawdziwy prąd (50%), zero (40%), **szpilkę 20-33A** (10%)
- Mitygacja: burst 3×3 + mediana + EMA — niewystarczająca pod obciążeniem
- **Fix**: przenieść odczyt ADC do ISR (TEZ) — infrastruktura gotowa, zmienne istnieją
- Poprawi: limiter prądowy (brak fałszywych cutoff), FOC (Clarke/Park potrzebuje dokładnych prądów)

## 2. WAŻNE — istotne ograniczenia

### D) FOC: `FOC_IQ_MAX` = 10A — hardware limit prądu
- `FOC_IQ_MAX = 10.0f`
- Niezależnie od duty, Iq target jest clampowany do 10A
- Jeśli silnik potrzebuje 20A pod obciążeniem → dostaje max 10A
- Ale to dotyczy tylko trybu FOC PI (nie voltage mode)

### E) RECOVER_RATE = 0.5/s — za wolne odzyskiwanie po limiterze
- `ILIMIT_RECOVER_RATE 0.5f`
- Po spike'u prądu factor spada → wraca z prędkością 0.0005/loop
- Od 5% do 100%: 0.95/0.0005 = **1900 iteracji ≈ 2 sekundy**
- Pod obciążeniem: ciągłe spike'i → factor nigdy nie wraca do 1.0

### F) KP_DOWN = 0.05 — za agresywne obcinanie
- Przy error = 5A ponad limit: factor -= 0.05 × 5 = -0.25 w jednym loop!
- 4 próbki powyżej limitu → factor = 0 → brak mocy

## 3. ŚREDNIE — dodatkowe ograniczenia

### G) Rampa 1200ms — ogranicza szybkość narastania
- Start od zera do pełnego duty zajmuje 1.2 sekundy
- Pod obciążeniem silnik potrzebuje szybszego startu
- Sprawdzenie: `ramp:200` (200ms)

### H) Brak BLOCK startup w SINUS — crawl na stojąco
- Na stojąco: `g_sine_speed_q16 = 0` → CRAWL (~1 obr.el./s) → bardzo wolny obrót
- Obserwer kąta nie wie gdzie jest rotor → wektor może być zupełnie obok
- BLOCK startuje natychmiast z pełnym momentem reaktywnie
- Sprawdzenie: przełączyć na BLOCK (`mode:block`) i spróbować ruszyć

### I) Stall freeze 200ms — pod dużym obciążeniem rotor hamuje
- Jeśli obciążenie spowalnia rotor → Halle przestają przychodzić → stall freeze
- Kąt zamiera → brak momentu → silnik stoi → deadlock
- Threshold 200ms może być za niski dla startu pod obciążeniem

## 4. DO WERYFIKACJI — mogą nie być problemem

### J) Offset fazowy PLL (naprawa `PLL_SIGN_FIX` czeka na test)
- Jeśli PLL_SIGN_FIX zadziała → err spadnie do ~0 → moment × 2.3
- To ONE zmiana może rozwiązać główny problem z mocą

### K) Wyświetlacz assist_level → max duty
- `assist_level / 15 × PWM_MAX_DUTY` — level 5/5 = rawValue 15 → 100%
- Ale level 3/5 = rawValue 9 → 60%. Sprawdź jaki poziom masz ustawiony

---

## Wyniki testów

### PLL_SIGN_FIX — ✅ DZIAŁA (bez obciążenia)
- err spadł z `[-18..-17]` do `[-8..0]` — ogromna poprawa
- Motor cichy, brak wiertarki
- Utrata momentu: z cos(64°)=44% do cos(15°)=97%
- **Pod obciążeniem**: silnik nie ruszył na trawie, na równym OK
- Przy ilim:99A płynęło 22A ciągłego — IRFB3607 się nagrzały (zapach), ale przeżyły
- Wniosek: moc jest, ale err=-4 (resztkowe) + fałszywe odczyty prądu + limiter = za mało momentu na trudnym terenie

---

## Zalecana kolejność testowania

1. **Najpierw**: wgraj `PLL_SIGN_FIX`, `so:0`, `gdbg` — sprawdź czy err spadł
2. **Potem**: `ilim:0` — wyłącz limiter prądu, próba jazdy
3. **Jeśli leci**: problem to fałszywe wyzwolenia limitera
4. **Jeśli nie leci**: `mode:block` i próba — czy BLOCK rusza?
5. **Status na serialu**: patrz czy leci `[ILIM] HARD CUTOFF` — to potwierdzi punkt C

Najważniejszy test to **PLL_SIGN_FIX + gdbg**. Jeśli err spadnie z -17 do ~0, moment wzrośnie ~2.3× i to samo może wystarczyć.
