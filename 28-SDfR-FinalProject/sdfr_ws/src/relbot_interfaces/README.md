# Package RELbot Interfaces
---
---
## Description

This package contains two different custom messages used in the package `brightness_detection` and `ball_detection` 

## Messages

### 1. Custom Brightness Detection Message
 - **Description**: This message contains two variables, one for the `brightness value` and the other for a string containing the key words `bright` or `dark` depending of the brightness.

 - **Message Structure**

   ```bash
   # My custom message for brightness status
   string brightness_status
   uint8 light_level
   ```

### 2. Bounding Box Message
 - **Description**: This message contains the data regarding a bounding box. It contains the `center points coordinates x and y`. The `width` and the `height`. It also has a `boolean` to check if the ball was found or not.

 - **Message Structure**

   ```bash
   # BoundingBox.msg
   float32 center_point_x
   float32 center_point_y

   float32 width
   float32 height
   bool ball_found
   ```