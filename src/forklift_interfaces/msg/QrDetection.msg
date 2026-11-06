# One QR code that the node decoded from a camera image. The node uses only the image
# pixels. It does not use the names of simulator entities.

std_msgs/Header header        # stamp = stamp of the source image, frame_id = camera optical frame

string payload                # raw decoded text, for example "BOX_A:SHELF_01"
bool valid                    # true only if the payload was parsed AND agrees with the box configuration
string rejection_reason       # empty if valid; if not valid, the reason for the rejection
string box_id                 # parsed box id (empty if the parse was not possible)
string destination_id         # parsed destination id (empty if the parse was not possible)

# Pose of the QR-code center in header.frame_id (optical convention: z forward, x right,
# y down). It comes from solvePnP with the known printed QR size and the camera_info intrinsics.
bool pose_valid
geometry_msgs/Pose pose

float32[8] corners_px         # u0,v0,u1,v1,u2,v2,u3,v3 in image pixels
float32 side_length_px        # mean edge length in pixels (it shows the visibility and the range)
float32 confidence            # 0..1 heuristic from the size, the reprojection error and the decode result
