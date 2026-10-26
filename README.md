# LMS / Meshmerize — Fresher Selection Tasks

Submission for the Robotronics Club Line Maze Solver selection tasks (9 tasks, 300 points).
The tasks target an ESP32 line-maze-solving robot for **Meshmerize, Techfest 2026–27**
(IIT Bombay): a 230 × 230 cm maze of 30 mm white lines on black, explored in a dry run
and then solved by the shortest route.

**Start with [`docs/NOTES.md`](docs/NOTES.md)** — the key design decisions and why I made them.

---

## Repository layout

```
README.md
BOM/
  Task4_BOM.xlsx                    Task 4  bill of materials (₹8,605 incl. contingency)
  power_block.png                   Task 4  power-block diagram
  Task5_Power_Budget.docx           Task 5  power budget, runtime, LiPo vs Li-ion
docs/
  NOTES.md                          Critical design decisions (start here)
  Task2_Datasheet_TB6612FNG.docx    Task 2  datasheet literacy, TB6612FNG vs L298N
  Task3_PID_Control.docx            Task 3  PID theory and choice of PD
  Task6_Electronics_Wiring.docx     Task 6  wiring, supply rails, troubleshooting
  wiring_diagram.png                Task 6  pin-level wiring diagram
  Task7_Wireless_Diagnostics.docx   Task 7  protocol choice and architecture
  wireless_diagnostics.png          Task 7  data-flow block diagram
  Task8_Maze_Solving.docx           Task 8  DFS vs wall rule, worked example, pseudocode
  maze_example.png                  Task 8  annotated example maze
  Task9_PID_Notes.docx              Task 9  how the code works, test results, PID tuning
  Project_Overview_and_Decisions.docx  overview and dated log of my decisions
src/
  task1/task1.py                    Task 1  centroid and motor-command limiting
  task8_maze/example_maze.py        Task 8  left-hand rule vs DFS on the example maze
  task9_line_follower/task9_line_follower.ino   Task 9  ESP32 PID line follower
```

## How to run the code

**Task 1 (Python 3, no libraries needed)**

```
cd src/task1
python task1.py
```
Prints the line position for five example sensor patterns (including the all-zero case)
and clamped motor commands, then runs its self-checks.

**Task 8 (Python 3, no libraries needed)**

```
cd src/task8_maze
python example_maze.py
```
Runs the left-hand rule and DFS on the example maze and prints every step.
Expected result: left-hand replay 240 cm, DFS + shortest route 120 cm (S → A → F → E).

**Task 9 (ESP32, Arduino IDE)**

1. Install the ESP32 board package in the Arduino IDE (Boards Manager → "esp32" by Espressif).
2. Open `src/task9_line_follower/task9_line_follower.ino`.
3. Select the board **ESP32 Dev Module**, choose the port, and upload.
4. Open the Serial Monitor at **115200 baud**.

With `TEST_MODE = true` (default) the program runs six hard-coded sensor frames and prints:

| Frame | Expected left, right |
|---|---|
| Centred | 150, 150 |
| Line to the right | 255, 39 |
| Further right | 255, 25 |
| Coming back | 144, 156 |
| Line lost | 120, −120 (spin right) |
| Found on the left | 97, 203 |

Set `TEST_MODE = false` to drive the robot: it calibrates for 3 s (sweep the sensors across
the line), then follows the line.

**Wiring for Task 9 (Appendix A):** sensors S1–S5 on GPIO 34, 35, 32, 33, 25; L298N
ENA 14, IN1 27, IN2 26, ENB 13, IN3 12, **IN4 4** (Appendix A says 11 — see Limitations).
Remove the ENA/ENB jumpers on the L298N.

## Assumptions

- Task 9 follows Appendix A: 5 analog sensors where the line is darker (higher voltage), an
  L298N driver and a 3S LiPo. Tasks 4–6 describe my own robot design (TB6612FNG, Pololu QTRX
  8-channel sensor, 2S LiPo) as asked for in "your BOM".
- Motor running current (~0.25 A) and stall current (~1 A, worst case 1.6 A) are estimates;
  the supplier does not list them.
- Buck converter efficiency is assumed to be 85%.
- Prices include GST and were read on 21 and 25 October 2026; they change often.
- The robot has no wheel encoders at this stage.

## Limitations

- **GPIO 11 cannot be used:** Appendix A puts IN4 on GPIO 11, which is wired to the ESP32's
  flash chip. The code uses GPIO 4 instead; hardware wired exactly to Appendix A must move
  that one wire.
- **GPIO 25 is ADC2**, not ADC1 as Appendix A says. It works in Task 9 because Wi-Fi is off.
- Task 9 was compiled and run on a PC with a small stand-in for the Arduino functions; it
  has not yet been tested on a physical robot.
- PID gains (Kp 50, Kd 60, base 150) are starting values and must be tuned on the real robot.
- Without encoders the robot cannot reliably recognise a junction it has visited before, so
  the DFS maze method needs encoders on real hardware; the left-hand rule is the fallback.
- Wi-Fi diagnostics (Task 7) are for practice only: Meshmerize requires wireless devices to
  be switched off near the arena.
- The README's path-reduction rule "LBR → S" is incorrect; the correct result is B.

## References

- Meshmerize 2026–27 problem statement (Techfest, IIT Bombay)
- [Toshiba TB6612FNG datasheet](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660)
- [STMicroelectronics L298 datasheet](https://www.makerguides.com/wp-content/uploads/2019/07/L298N-Datasheet.pdf)
- [Espressif ESP32 GPIO and ADC reference](https://docs.espressif.com/projects/esp-idf/en/release-v6.0/esp32/api-reference/peripherals/gpio.html)
- [Pololu QTRX-MD-08A / 08RC reflectance sensor arrays](https://www.pololu.com/product-info-merged/4448)
- [SmartElex RLS-08 user manual](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2018/06/SmartElex-RLS08_manual-1.pdf)
- [Vishay TCRT5000 datasheet](https://www.vishay.com/docs/83760/tcrt5000.pdf)
- [Pololu 6 V HP micro metal gearmotor specifications](https://www.pololu.com/product/999/specs)
- [Handsontec GA12-N20 6 V 300 RPM motor](https://handsontec.com/index.php/product/ga12-n20-6v300rpm-geared-mini-dc-motor/)
- [TI DRV8833 product page](https://www.ti.com/product/DRV8833)
- [ESP32 power consumption (Espressif figures, via JLCPCB)](https://jlcpcb.com/blog/esp32-power-consumption-explained)
- [Pololu 3pi maze-solving guide](https://www.pololu.com/docs/0J21/8.a)
- Supplier pages for prices are linked in `BOM/Task4_BOM.xlsx`.

## Acknowledgement

I used an AI assistant (Claude) for research, calculations, drafting the documents and
writing code, and directed, reviewed and questioned all of it. My decisions and my own code
contribution are listed in `docs/NOTES.md` 
