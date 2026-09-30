# Moon Or Bust!

An accurate orbital simulation that is made with raylib that lets you land and orbit around the moon. Inspired heavily by KSP, the game copies much of its ideas and aspects in terms of navball, camera,  controls, and even time warp! .

> **Note:** Keep in mind that landing can be a bit scuffed as it was more or less hard coded due to time limitations.

| Action            | Key       |
|-------------------|-----------|
| Roll              | `E` / `Q` |
| Pitch             | `W` / `S` |
| Yaw               | `A` / `D` |
| Lower throttle    | `L-Ctrl`  |
| Increase throttle | `L-Shift` |
| Set to max        | `Z`       |
| Set to 0          | `X`       |
| Increase warp     | `]`       |
| Decrease warp     | `[`       |

You can also timewarp, although it's not recommended for long periods of time, as the orbit eventually deviates due to floating point precision. The integration method is velocity verlet, since it's very simple and fast for me to implement while still maintaining accuracy.

To build it, all you have to do is run the batch file. Unfortunately, you can only build it with Mingw for Windows. If you can't build it, a Windows release is available under releases.

Hope you enjoy, and feedback would be appreciated!