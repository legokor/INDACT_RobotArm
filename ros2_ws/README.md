# ROS2 használata

## Telepítés

A ROS2 Humble disztribócióját használjuk, ami az Ubuntu 22.04 LTS disztribúciójához illeszkedik.

Akinek Ubuntu 22.04 van a gépén, annak csak a ROS-t kell telepítenie, a többieknek a Linux-ot is be kell állítania. Windows-hoz a WSL2-t ajánlom. A WSL telepítéséhez a következő linkeken lehet útmutatót találni:
[Install WSL | Microsoft Learn](https://learn.microsoft.com/en-us/windows/wsl/install),
[Windows Subsystem for Linux (WSL) | Ubuntu](https://ubuntu.com/desktop/wsl).
A WSL-en kívül lehet még egyéb virtuális gépeket vagy akár dual-boot-ot is használni.

A ROS 2 Humble telepítéséhez a következő linken lehet útmutatót találni:
[Installation - ROS 2 Documentation](https://docs.ros.org/en/humble/Installation.html).
Akinek az Ubuntu 22.04 be van állítva, az követheti az ahhoz releváns útmutatót:
[Ubuntu (Debian packages) - ROS 2 Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
A ROS Desktop változatát érdemes telepíteni.

## Robot modell készítése (URDF)

A robot modellezéséhez a URDF formátumot használjuk. Ehhez dokumentáció a következő linken található:
[urdf - ROS Wiki](http://wiki.ros.org/urdf).
Az XML formátum ponotsabb leírását a
[urdf/XML - ROS Wiki](http://wiki.ros.org/urdf/XML)
oldalon lehet megtalálni.

A modell készítése közben a urdf_tutorial csomaggal lehet ellenőrizni. Ennek telepítése és használata:

```bash
# Csomag telepítése
sudo apt install ros-humble-urdf-tutorial

# Futtatás
source /opt/ros/humble/setup.bash
ros2 launch urdf_tutorial display.launch.py model:={TELJES FÁJL ELÉRÉSI ÚT}.urdf
```

A parancs futtatása után az RViz vizualizációs eszköz nyílik meg. Ebben látható a robot modellje, és a csuklók mentén mozgatható egy külön ablakban található csúszkák segítségével.
