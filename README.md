# SOA Robot Dealer: An Autonomous Big/Small Dice Game

> **TECHIN 517 final project.** A bi-manual SO-101 robot that runs a complete
> Big/Small (Sic-Bo style) dice betting game on its own: it watches the player
> place a chip, rolls and reads the dice, decides who wins, and then either pays
> the player out or collects the losing chip, round after round.
---

## 1. What it does

The robot acts as the dealer for a simple two-dice Big/Small game and runs it
as a continuous loop, fully unattended:

1. **Wait for a bet.** An overhead Intel RealSense camera watches two betting
   zones (SMALL and BIG). The player places an orange chip on one of them.
2. **Lock the bet.** Once a single zone holds an orange chip steadily for ~2 s,
   that bet is locked in. The locked value is what settlement uses, even if the
   player moves the chip afterward (anti-cheat).
3. **Roll and read the dice.** The arms run a scripted dice-handling routine,
   then a YOLO detector reads the two dice faces and sums them. Total ≥ 7 is
   **BIG**, otherwise **SMALL**.
4. **Resolve.** If the bet matches the dice, the player **wins**; otherwise the
   player **loses**.
5. **Pay out or collect.**
   * **Win** → the robot dispenses chips with a scripted trajectory (stays under
     ROS control).
   * **Lose** → the robot collects the player's chip using a learned **ACT
     (Action Chunking Transformer)** vision-action policy run natively in
     `lerobot`.
6. **Reset.** A short buffer gives the player time to clear chips and re-bet,
   then the loop repeats.

The whole game is driven by a single orchestrator node
(`soa_apps/orchestrator.py`) launched with `phase:=round`.

---

## 2. Demo video

**Video demo:** `https://drive.google.com/file/d/1gqPoofl5bv9lRdDnXwjeGp93WpBHYT1W/view?usp=drive_link`

---

## 3. System architecture

```
                 ┌─────────────────────────── orchestrator (phase:=round) ───────────────────────────┐
                 │                                                                                    │
 overhead cam ──▶│  bet detection (HSV orange, settle timer)  ─▶  dice routine (CSV FK replay)        │
 (RealSense)     │                                                       │                            │
                 │                                                YOLO dice reader  ─▶  resolve W/L    │
                 │                                                                          │          │
                 │   WIN ─▶ chip_dispense (CSV FK, ROS)        LOSE ─▶ stop bringup ─▶ ACT (lerobot)   │
                 │                                                          │  native, frees USB ports │
                 └──────────────────────────────────────────────────────── ▼ ────────────────────────┘
                                                              restart bringup next round
```

Key design point: `lerobot` opens the motor and camera devices directly, while
ROS `ros2_control` also wants exclusive access to them. Because the ACT policy
starts and ends at the home pose, the orchestrator does a **hard handoff**: it
tears down the ROS bringup to release the USB devices, runs the native `lerobot`
policy, then brings ROS back up for the next round. No `rosetta` bridge is used.

### Hardware

| Component | Detail |
|---|---|
| Arms | Bi-manual SO-101 followers (Feetech servos) |
| Follower ports | left `/dev/ttyACM0`, right `/dev/ttyACM1` |
| Leader (teleop / ACT) | left `/dev/ttyACM2` |
| Overhead camera | Intel RealSense D435i, serial `241122070943` |
| Wrist camera | `/dev/video0` (used by the ACT policy) |

### Software

ROS 2 Humble, `ros2_control` + `feetech_ros2_driver`, `lerobot` (ACT), Ultralytics
YOLO, OpenCV, `cv_bridge`. Custom ROS 2 packages under `src/soa_ros2/`:
`soa_apps` (the orchestrator and game logic), `soa_bringup`, `soa_description`,
`soa_interfaces`, `soa_functions`, `soa_moveit_config`, `soa_teleop`,
`soa_sim2real`.

---

## 4. Safety, robustness, and anti-cheat features

These are built into `orchestrator.py` and map to the evaluation priorities:

* **Bet is locked at detection.** Once read, the bet never changes, so a player
  moving the chip after the roll cannot alter the outcome.
* **Double-bet rejection.** If chips appear on both SMALL and BIG, the robot
  refuses to proceed and prompts the player to bet on only one zone.
* **Chip-removal alarm.** At settlement the robot re-checks the tray. If the chip
  was removed early (no chip anywhere on the tray), it raises an alarm and stops
  the game process rather than acting on a missing bet.
* **Settle-time gating.** A bet only locks after the chip has been steady for
  ~2 s, so the robot never reacts to a chip still being placed.
* **Tilted-die handling.** A cocked die can show two faces; the dice reader
  spatially de-duplicates detections so it always resolves to exactly two dice.
* **Home-anchored handoff.** The ACT policy begins and ends at home, so the brief
  loss of holding torque during the ROS↔lerobot handoff happens in a safe pose.
* **Inter-round buffer.** A fixed buffer between rounds gives a human time to
  clear and reset the board without the robot moving.

---

## 5. Repository structure

```
.
├── .devcontainer/
│   ├── Dockerfile
│   └── devcontainer.json
├── src/soa_ros2/
│   ├── soa_apps/soa_apps/orchestrator.py      # main game orchestrator
│   ├── soa_bringup/ ...                        # launch + hardware params
│   ├── soa_description/ ...                     # URDF, controllers
│   └── ...                                      # other soa_* packages
├── LICENSE
└── README.md
```

> The orchestrator lives at
> `src/soa_ros2/soa_apps/soa_apps/orchestrator.py`.

---

## 6. Quantitative results

### 6.1 Per-stage accuracy

| Stage | Trials | Correct | Accuracy |
|---|---|---|---|
| Bet detection (SMALL vs BIG) | `<n>` | `<n>` | `<%>` |
| Dice reading (correct sum) | offline val set | — | mAP50 = **94.5 %**, P = 95.6 %, R = 85.4 % (YOLOv8n, 50 epochs) |
| Chip-collect ACT (successful grasp + place) | `<n>` | `<n>` | `<%>` |
| Chip dispense (scripted) | `<n>` | `<n>` | `<%>` |

> Bet detection, ACT, and dispense numbers should be filled in after running repeated live rounds with `test_bet.py` and manual logging.

### 6.2 End-to-end

| Metric | Value |
|---|---|
| Full-round success rate | `<%>` |
| Rounds before a reset/maintenance is needed | `<n>` |
| Mean round duration | `<s>` |


## 7. Pretrained models


| Model | Purpose | Link |
|---|---|---|
| Chip-collect ACT policy | learned chip pick-and-place (LOSE branch)  |
| Dice-recognition YOLO | reads the two dice faces |

After downloading, place them so the paths in `orchestrator.py` resolve:

```
~/techin517/craps-chip-collect-v3-act-train/checkpoints/last/pretrained_model   # ACT
~/techin517/dice-recognition/runs/detect/train/weights/best.pt                  # YOLO
```

---

## 8. Setup

### 8.1 Devcontainer (recommended)

The repo ships a `.devcontainer/` so the environment is reproducible. With Docker
and the VS Code Dev Containers extension installed:

1. Clone the repo and open it in VS Code.
2. "Reopen in Container". The Dockerfile installs ROS 2 Humble, `lerobot`,
   Ultralytics, OpenCV, and the RealSense / Feetech dependencies.
3. The container is launched with USB device access so the arms and cameras are
   visible inside it.

> **TODO(team):** confirm the container passes `--device` / udev rules for
> `/dev/ttyACM*` and the RealSense, and document any host-side udev setup.

### 8.2 Build the workspace

```bash
cd ~/techin517/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 8.3 Hardware configuration

Edit `src/soa_ros2/soa_bringup/config/bi_soa_params.yaml` so the ports match your
wiring (left follower `/dev/ttyACM0`, right follower `/dev/ttyACM1`, left leader
`/dev/ttyACM2`). Confirm the RealSense serial in `orchestrator.py`
(`241122070943`) and in the camera config.

> USB enumeration can change on replug/reboot. If ports drift, add udev rules to
> bind each arm to a stable name.

---

## 9. Usage

### Run the full game (single command)

```bash
ros2 run soa_apps orchestrator --ros-args -p phase:=round
```

The orchestrator brings up the robot itself (`controller:=jtc`), waits for a bet,
rolls and reads the dice, resolves, pays out or collects, then loops. Press
`Ctrl-C` to stop; the bringup is torn down automatically on exit.

### Run a single stage (debugging)

```bash
ros2 run soa_apps orchestrator --ros-args -p phase:=dice_sequence   # dice handling only
ros2 run soa_apps orchestrator --ros-args -p phase:=chip_dispense   # payout only
ros2 run soa_apps orchestrator --ros-args -p phase:=chip_collect    # ACT collect only
```

### Tune the bet detector

```bash
python3 tools/test_bet_detection.py
```

Live view with HSV trackbars and the over-threshold timer, for tuning the orange
range against the yellow tray. Copy the values into `orchestrator.py`.

### Record FK waypoints

```bash
python3 tools/record_fk.py --left ~/techin517/craps_left.csv \
                           --right ~/techin517/craps_right.csv
```

Move the arms to each pose (via leader teleop or hand-guiding) and press Enter to
capture a row.

---

## 10. Generalization notes

<!-- TODO(team): replace with what you actually observed in testing. -->

* **Related objects:** `If we only test on ACT policy for picking up the chip from the plate, other standard casino chips also works for the policy, but for the whole pipeline, since the OpenCV canmera needs to recognize the orange color, it still needs the specific GIX chip we designed for this project`
* **Lighting:** `The whole pipeline have the same performance under differenct lightning conditions (cold/warm, bright/dim)`
* **Clutter / random objects:** `The plate allows 5mm offset, but no obvious tilted angle; The cluttered backgound (for example, putting random object in the scene or around the plate doesn't affect the whole pipeline or the policy itself)`
* **Related environments:** `It can transfer to a different background but the position of the dice tower, the trailer and the plate needs to be fixed to the same relative position`

---

## 11. Team contributions

| Member | Contributions |
|---|---|
| `Phoenix Hua` | `YOLO dice recognition (model implementation, ROI calibration, 8× upscale fix); orchestrator integration (chip detection → FK sequence → YOLO readout → BIG/SMALL logic); ROS 2 node architecture & system pipeline design ` |
| `Jason Jin` | `FK waypoint recording for all 2 dice pick positions; CSV motion sequence design & bimanual timing; Right-arm cup-holding & dice pourting sequence recording; ACT policy training; Orchestrator integration: payout pipeline` |
| `Xiangpeng Yu `| `Dice tower physical design & fixed rail fabrication; Dice landing position measurement & verification; Hardware setup, calibration & repeatability testing and evaluation; ACT policy training` |

---

## 12. License

<!-- TODO(team): pick and add the matching LICENSE file. -->

This project depends on **Ultralytics YOLO, which is AGPL-3.0**. AGPL-3.0 is a
strong copyleft license, so to comply with all dependency licenses this repo
should be released under **AGPL-3.0** as well (or you must obtain an Ultralytics
commercial license, or remove the YOLO dependency). Other major dependencies
(ROS 2, `lerobot`, OpenCV) are Apache-2.0, which is compatible with AGPL-3.0.

This is general information, not legal advice. Add a top-level `LICENSE` file with
the chosen license text.

---

## 13. Acknowledgments

Built on ROS 2 Humble, `ros2_control`, Hugging Face `lerobot` (ACT), and
Ultralytics YOLO, for TECHIN 517.
