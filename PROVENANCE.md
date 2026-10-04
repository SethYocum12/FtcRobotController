# Provenance and validation

Upstream: https://github.com/Beta8397/virtual_robot
Pinned commit: 7105cfa534026a62df5176c8a7b76e7196b61b31
The simulator directory is an independent Git clone. Lab changes are recorded in patches/simulator.patch: default to MecDynamicBot and fix Pinpoint setHeading() so it actually applies the new position.

The lab root excludes the nested simulator repository from its own Git history. To reconstruct it: clone the upstream into simulator, checkout the pinned commit, and apply patches/simulator.patch. Preserve upstream licenses. The existing simulator clone is already set up.

Pedro core/revhub 3.0.1 and Ivy core/pedro 1.1.1 jars in dependencies were obtained from this computer's existing robot build cache (Android AAR classes extracted where needed). They replace the upstream bundled Pedropathing.jar on the desktop classpath. These are third-party libraries, not lab-authored code; preserve their upstream licensing when redistributing. This lab has not been published.

Java: existing C:/Program Files/Java/jdk-17. JavaFX: Maven Central 17.0.12 Windows artifacts. Gradle: 9.1.0, with a generated wrapper fallback. The launcher uses an existing cached 9.1.0 installation when available; dependencies/cache stay in this lab's .gradle-user. No global Java configuration was changed.

Validated on 2026-10-04:
- Desktop compilation with the exact robot Pedro/Ivy versions: passed.
- Actual JavaFX controller, OpMode loop and dyn4j physics: passed.
- Pinpoint heading reset preserves position: passed.
- LearnForward: (72.000, 83.737), heading 90.00 degrees.
- LearnLeft: (60.322, 72.002), heading 90.03 degrees.
- LearnTurn: (72.001, 71.999), heading 178.91 degrees.
- Upstream Ivy example moved and STOP zeroed all four motors: passed.
- Independent FTC SDK 12.0.0 Android app with the same shared lesson sources: assembleDebug passed.

Detailed logs are in logs/. JavaFX reports unnamed-module and FXML version warnings; they did not prevent loading or executing the tested controller. The Android build reports Gradle deprecations and native-library stripping warnings, but successfully produces the APK.

The desktop launcher keeps compiled class directories on its classpath because upstream OpMode discovery did not work from the packaged application JAR alone. The application window also scales to the available screen so controls remain accessible on smaller laptop displays. These local UI changes are included in the recorded simulator patch.
