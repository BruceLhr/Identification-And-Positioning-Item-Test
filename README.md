# Identification-And-Positioning-Item-Test
This's a test Item, not full item. May be have full item in the future.

In order to catch mark object with a robot gripper, the full item have camera calibration, moving object detection, object identification and object location.
The full item may be use R-CNN instead of the current Moving object detection method and Object identification method.

This test item is base on Opencv:
Camera calibration is ZhengYou Zhang camera calibration.
Moving object detection is background substract.
Object identification is Neural networks with HSV model.
Object location is SURF.

Camera calibration will detect a corner of first pic as origin of robot gripper, store the direction difference betwenn the first pic and the second pic as robot x-axis.

Background substract need to use GMM in order to reduce noise.

Neural networks with HSV model 
