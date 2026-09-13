# ERC perception branch

This package reads the front RGB camera, recognizes the randomized shelf labels,
identifies the requested book's active row, publishes the two scoring topics,
and saves annotated live camera frames. It does not start Gazebo or move the
robot. Navigation and manipulation packages are outside this update.

## What changed

- `DigitRecognizer` is now used by the camera node. Column selection follows
  the **visible digit**, not its position from the left.
- A target book must be inside that labelled column. A same-colour book in a
  neighbouring column cannot substitute for a missing target.
- All four distinct book colours must be visible in the target column before
  numbering its active rows **1–4, from top to bottom**. Empty top/bottom shelf
  compartments are excluded. Missing or ambiguous books produce no row result.
- Five consecutive fresh processed frames must agree on the visible label
  order and, for a row answer, the requested colour and row. Missing detections,
  changed answers, long gaps and reset camera timestamps restart confirmation.
- The node processes at most 5 frames/s by default and publishes confirmed
  answers at most once per second. It does not continuously replay a cached
  answer after losing the target. The Int32 topics do not encode invalidation;
  consumers should not treat old messages as a current pose estimate.
- A confirmed column can be published and photographed before its book is
  confirmed. Successful evidence saves are limited to once per result per
  process. Image save errors are reported and retried.
- Launch arguments have explicit types. Runtime dependencies include the
  launch packages, `ament_index_python`, and `erc_description` for templates.

## Interface

| Direction | Topic | Message |
| --- | --- | --- |
| Input | `/head_front_camera/head_front_camera/color/image_raw` | `sensor_msgs/msg/Image` |
| Output | `/erc/shelf_column_identification` | `std_msgs/msg/Int32`: observed target label, 1–5 |
| Output | `/erc/shelf_row_identification` | `std_msgs/msg/Int32`: active book row, 1–4 |

Input uses sensor-data QoS. Outputs use reliable, volatile publishers. The
requested target is set at launch; restart the node to change it.

```bash
docker exec -it erc_sim bash -lc 'source /opt/ros/humble/setup.bash && source /opt/erc_ws/install/setup.bash && ros2 launch erc_perception perception.launch.py shelf_column_number:=3 book_colour:=blue'
```

The package also provides `solution.launch.py` as a **perception-only alias**.
The whole-team solution launch remains an integration responsibility. Neither
launch file starts a simulator, navigation, manipulation, or a complete trial.

Additional launch arguments:

| Argument | Default |
| --- | --- |
| `output_directory` | `/opt/erc_ws/erc_images` |
| `template_directory` | Empty: resolve `erc_description` package share |
| `camera_topic` | Front RGB topic above |
| `confirmation_frames` | `5` |
| `processing_rate` | `5.0` Hz |

Templates are the static artwork `models/number_marker/textures/1.png` through
`5.png` from `erc_description`. The node never reads a randomized seed, spawned
model names, world poses, or a ground-truth label order to make decisions.

## Viewing requirements and limits

This is a conservative implementation for a roughly frontal view. It uses the
known shelf proportions to validate apparent label spacing and book sizes.
It needs at least three aligned, readable neighbouring labels, with the target
between two neighbours. For the leftmost or rightmost column, all five labels
are needed to establish the outer boundary. All four target-column books must
fit in the image to assign their active row reliably.

Digit glyphs must be at least 8 pixels high. Strong perspective, glare, blur,
partial occlusion, a very close camera, or a view facing away from the shelf can
prevent detection. Colour and geometry thresholds still need validation on
actual simulator camera frames. A no-result message is preferable to a guessed
label/row. Passing a synthetic check does not establish real-camera accuracy.

The two Int32 outputs are the competition scoring interface. This package does
not yet supply a metric 3D navigation/grasp pose, a TF transform, bin detection,
or an autonomous scan behaviour. Those require an agreed additional interface
and, for position, depth/camera calibration. Do not describe this package as a
complete robot solution.

## Install the downloaded update

Stop an older perception process with Ctrl+C before replacing its source.
Save the updated archive as `~/Downloads/erc_perception_install_bundle.tar.gz`.
Run this on the **host terminal**, not inside Docker:

```bash
update_dir=$(mktemp -d /tmp/erc-perception-update.XXXXXX)
tar -xzf "$HOME/Downloads/erc_perception_install_bundle.tar.gz" -C "$update_dir"
bash "$update_dir/erc_perception/tools/install_and_check.sh" "$HOME/erc_sim_2026_team"
```

The installer checks that the repository is on `perception`, backs up the
existing package to a new `~/erc-perception-backup.*` folder, and copies only
`src/erc_perception`. It builds the package, runs package tests, and exercises
ROS publishers with synthetic camera frames on uniquely named test topics.
It starts the existing `erc_sim` container if needed. It does not launch
Gazebo, edit another package, commit, or push.

## Validation status for this update

Completed in the review workspace:

- 15 decision/stability tests passed, including 600 target selections across
  all 120 label permutations and 96 row selections across all 24 colour orders.
- Missing targets, neighbouring books, duplicate candidates, missing labels,
  nonuniform label gaps, cropped shelves, oversized red objects, and temporal
  disagreement are covered.
- Python source compilation and package XML parsing passed.

Pending in the provided Docker environment:

- The eight image tests in `test/test_vision.py`, including recognition using
  the actual number artwork, dark backgrounds, colour permutations, missing
  targets, blank inputs and cropped images.
- The existing ament lint/docstring checks and a ROS build/launch check.
- `tools/check_ros.py`: blank images produce no answer; a synthetic shelf
  produces column 5 / row 4; both topics and temporary image saving work;
  answers stop after the view becomes blank.
- A live Gazebo camera run with visually checked label, colour, row and saved
  boxes. Repeat with different randomized layouts before claiming accuracy.

OpenCV and ROS were not available in the review workspace. Its network policy
blocked installing OpenCV, so the pixel and ROS tests were supplied but not
reported as passing. The earlier reference-strip test from the user passed for
the previous digit reader; it is not a substitute for testing this integration.

To repeat checks after installation:

```bash
docker exec -it erc_sim bash -lc 'source /opt/ros/humble/setup.bash && source /opt/erc_ws/install/setup.bash && cd /opt/erc_ws && colcon test --packages-select erc_perception --event-handlers console_direct+ && colcon test-result --test-result-base build/erc_perception --verbose && python3 src/erc_perception/tools/check_ros.py'
```

## Live validation and evidence

Use one existing simulation instance. The camera must face the shelf with the
viewing requirements above; this can be arranged during a development test
without waiting for the navigation branch. Start the perception launch in a
separate terminal. Read the result topics from two additional terminals:

```bash
docker exec -it erc_sim bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo /erc/shelf_column_identification --once'
docker exec -it erc_sim bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo /erc/shelf_row_identification --once'
```

An echo waits if no confirmed result is available. Compare the values with the
camera view, not with a simulator seed or a model name. The node logs whether
it needs labels, all four books, or more confirming frames.

With the existing Docker bind mount, images appear on the host in
`~/erc_sim_2026_team/erc_images`. Each JPEG has a UTC timestamp, a column box,
and a book box when its row is confirmed. A JSON sidecar records the ROS image
timestamp, source frame, visible label order and bounding boxes. The filename
uses `rowpending` for column-only evidence. These boxes are image-space
estimates, not 3D localization.

Synthetic tests write only temporary files outside `erc_images`; those files
are removed at the end and must not be presented as trial evidence. Only
images from an actual live competition-camera run satisfy the live evidence
requirement. Keep real validated trial evidence and label any development
fixtures honestly.

Once the automated and live checks pass, review and commit only this package:

```bash
cd ~/erc_sim_2026_team
git diff --stat
git add src/erc_perception
git diff --cached --stat
git commit -m "Use visible shelf labels for confirmed perception results"
git push origin perception
```

References checked for implementation details:
[OpenCV contour hierarchy](https://docs.opencv.org/4.x/d9/d8b/tutorial_py_contours_hierarchy.html).
Requirements were checked against the supplied ERC Phase 1 guide and team plan;
row numbering here means the four occupied rows from top to bottom.
