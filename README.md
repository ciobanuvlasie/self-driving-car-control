# Self-Driving Car Control — NXP Cup 2026

Software de control și navigație autonomă pentru un vehicul de curse miniaturizat, dezvoltat în cadrul competiției **NXP Cup 2026**.

Proiectul implementează un sistem embedded complet, de la achiziția datelor de la cameră până la comanda în timp real a motorului și a direcției.

---

##  Algoritmul de control

### Tip: Controler P (Proporțional)

Sistemul de direcție folosește un **controler proporțional (P)**, în care unghiul transmis servomotorului este calculat direct pe baza erorii detectate față de centrul pistei:

```
unghi_servo = Kp × eroare
```

### Metoda celor două puncte virtuale

Elementul central al algoritmului este conceptul de **două puncte virtuale** , două poziții de referință definite în imaginea capturată de cameră, la distanțe diferite față de vehicul:

| Punct virtual | Rol |
|---|---|
| **Aproape** | Detectează centrul pistei în zona imediată |
| **Departe** | Anticipează direcția traseului la distanță |

Fiecare punct calculează independent centrul local al pistei. **Eroarea de control** reprezintă diferența dintre cele două centre:

```
eroare = centru_departe − centru_aproape
```

Această diferență codifică **curbura anticipată a traseului**: dacă cele două centre sunt aliniate, mașina merge drept; dacă diferă, înseamnă că urmează o curbă și corecția se aplică proactiv, înainte ca deviația să devină critică.

Comanda finală:

```
unghi_servo = Kp × (centru_departe − centru_aproape)
```

Abordarea elimină reacția întârziată specifică unui control pur reactiv și permite vehiculului să anticipeze traseul în loc să îl corecteze după fapt.

---

## ⚙️ Arhitectura sistemului

Proiectul este construit pe **NXP SDK** și dezvoltat în **MCUXpresso IDE**, cu cod scris integral în **C**.

```
self-driving-car-control/
├── source/        # Logica principală: procesarea imaginii, algoritm de control
├── include/       # Headere și definiții globale
├── drivers/       # Drivere periferice NXP (PWM, UART, GPIO, camera)
├── board/         # Configurare hardware specifică plăcii
├── device/        # Fișiere de inițializare microcontroller
├── CMSIS/         # Bibliotecă ARM Cortex-M (standard DSP/Core)
├── startup/       # Vectori de întrerupere și inițializare la boot
├── utilities/     # Funcții auxiliare (debug, logging)
└── doc/           # Documentație tehnică
```

**Stack tehnologic:**
- **Limbaj:** C (94%)
- **Build system:** Makefile
- **IDE:** MCUXpresso (Eclipse-based), configurat prin `nxpcup.mex`
- **Standard:** CMSIS (Cortex Microcontroller Software Interface Standard)
- **Comunicație:** UART pentru debug și telemetrie

---

## 🎬 Demo

[![Demo NXP Cup 2026](https://img.shields.io/badge/▶%20Demo%20video-YouTube-red?style=for-the-badge&logo=youtube)](https://www.youtube.com/watch?v=8tY1c-_ZurY)

---

*NXP Cup 2026 — Autonomous Model Car Challenge 🏁*
