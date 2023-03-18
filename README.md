# INDACT robotkar programkódja

## STM32F429_StepperMotorControlling Library:
A könyvtárban a létetőmotorok használatához szükséges függvények és struktúrák szerepelnek.
- enum MovementCommands: Itt vannak felsorolva a robotkar által értelmezett parancsok.
- struct GPIO_PIN: GPIO lábak könnyű hivatkozására.
- struct ToolPosition: A megfogó pozícióját jegyzi. Segítségével helyzeteket lehet meghatározni a térben, ahova később vissza lehet térni.
- struct StepperMotor: Egy létetőmotor állapotát, a hozzátartozó timer referenciáját és az engedélyező és irány állító GPIO lábak azonosítóját tárolja.
- fc setMotorDir(): Megvizsgálja, hogy az általunk beállítani kívánt motorirány le van-e tiltva. Ha nem, akkor beállítja azt.
- fc setAllMotorDirTowardsDesiredPos(): A jelenlegi pozícióból és a következő pozícióból kiszámítja, hogy a motorokat merre kell forgatni a cél eléréséhez. Egyben be     is állítja az irányokat.
- fc startMotor(): Egy motor indításához szükséges függvény. Csak akkor indítja el a motort, ha nincs letiltva az irány.
- fc stop(All)Motor(): Megállítja a motort és átírja az állapotát ennek megfelelően.
- fc controlMotor_viaGPIO(): Az argumentumban megadott két gombnak megfelelően kapcsolja be a motort. Az egyik a pozitív irányba, a másik negatív irányba indítja. Ha     mindkettő nyomva van, akkor nem indul el a motort.

### STM32F429ZIT6U_StepperMotorControlling_Library hozzáadása a projekthez:
1) A projektben hozz létre egy mappát a könyvtár nevével és ebbe a mappába másold bele a könyvtár .h és .a kiterjesztésű fájljait.
2) Project > Properties > C/C++ General > Paths and Symbols > Includes > Add... .
   A Variables gombra kattintva keress rá és válaszd ki a ProjDirPath parancsot. Az OK gomb után bezáródik az előbb felugrott ablak, a szövegmezőbe pedig be van          helyettesítve a parancs. Egy perjel után írd be a most létrehozott mappa nevét:
   ${ProjDirPath}/StepperMotorControl_library.
   Pipáld be az "Add to all configurations" lehetőséget és mielőtt az OK gombra kattintanál másold ki a vágólapra a szövegmező tartalmát.
3) Project > Properties > C/C++ General > Paths and Symbols > Library Paths > Add... .
   Ide másold be a szöveget, megint pipáld be az "Add to all configurations" lehetőséget és OK. Végül "Apply" és "Apply and close".
4) Project > Properties > C/C++ Build > Settings > MCU GCC Linker > Miscellaneous menüben a jobb oldali kis ikonok közül kattints az Add... lehetőségre, azon belül a      Workspace... gombra. Itt válaszd ki a projektben lévő .a kiterjesztésű fájlt és nyomd meg az Ok gomot, majd"Apply",  "Apply and Close".
5) Ne felejtsd el includeolni a .h fájlt a main.c-ben.

## Karcsi STM32CubeIDE projekt:
- A projekt egy STM32F429I-DISC1 Discovery board-ra készült. Az .ioc fájlban hozzáadtam a projekthez a Middleware/FREERTOS menüben az operációs rendszert.
  A FreeRTOS (free real-time operating system) egy olyan mikrokontrollerekre készült OP rendszer, ami megkönnyíti a feladatok (task) ütemezését, ezzel
  meggyorsítja a fejlesztési időt.
  - LED Indicating Task: Ha hiba adódik valahol a programban, a task a kártyán lévő piros LEDet kezdi villogtatni, ezzel jelezve a gondot. Ha nincs hiba, egy zöld led     villog lassan. Jelentősége debugoláskor van.
  - **_GPIO Controlling Task_**: A task két részből áll: az első egyszer fut le a task indulásakor, a második fele egy végtelen hurokban ismétlődik. Az első részben       inicializálom a motorokat és elvégzem az úgynevezett homing-olást. Ennek lényege, hogy a kar minden tengelyét kiviszem a nulla állapotba és ebben a helyzetben         frissítem a pozíciót számon tartó változót. Ezek után ha mozgatom a kart már tudni fogom, hogy a koordináta-rendszerben pontosan hol járok. A végtelen hurokban a       controlMotor_viaGPIO() függvény meghívásával motoronként 2 gombbal irányítom a kart.
  - Wifi Controlling Task: A karhoz van egy weboldal, amin keresztül utasításokat lehet kiadni. Az MCU és a böngésző között egy wifi alapú kommunikáció biztosítja a       kapcsolatot. A wifi modul UART-on keresztül küldi az utasításokat az MCU-nak. A programban az utasítások egy, a freeRTOS-ban implementált, úgynevezett Queue-ba         kerülnek (gyakorlatilag egy stack struktúra), amiből az utasítások kivételével lehet azokat végrehajtani.
    A task ezt a működést implementálja: Induláskor lefut a motor inicializáló és homingoló rész, majd a végtelen ciklusban a queue-ból várja és hajtja végre az           utasításokat. Adott időpillanatban csak egy utasítást képes futtatni! A weboldal és a kommunikáció másik oldali implementációjához nézd meg az ahhoz tartozó           [branch](https://github.com/legokor/INDACT_RobotArm/tree/wifi)-et.
  - **_Simonyi Konferencia Demó Task_**: A konfra készült egy látványosságnak szánt task. Ebben a motor egy előre megírt pályát követ végtelen ciklusban. Igyekeztem       úgy megírni a pályát, hogy ha kitesszük standolni, akkor se a bámészkodó emberekbe, se a standolókba ne akadjon bele a robot. Mivel a task folyamatosan járatja a       motorokat és még nincs használatban a normális táp, ezért figyelni kell a tápegység melegedésére. Nem szerencsés hosszú időn keresztül járatni a kart.


- Mivel jelenleg nincsenek enkóderek felszerelve a karra, a pozíció meghatározására timer megszakításokat használok: A léptetőmotorokat PWM jellel kell szabályozni,
  amiket timerekkel állítok elő. Minden egyes PWM jel felfutó élre a timer megszakítást kér. A megszakítási rutinban az adott timerhez tartozó változót növelem
  vagy csökkentem annak függvényében, hogy a motor merre megy. A megoldás nem túl ideális, de jelenleg teljesen működőképes.
- A végálláskapcsolók is megszakítást generálnak minden kapcsoláskor. A GPIO megszakítási rutinban az EXTI line-nak megfelelően korlátozom az adott motor mozgását.       Erre (érthető módon) azért van szükség, hogy semmilyen program vagy felhasználó ne tehessen kárt a karban azzal, hogy a megengedettnél tovább hajtja a motorokat.

## XX. Simonyi Konferencia:
Megírtam a kódot amivel kivisszük standolni Karcsit. A program két taskot foglal magába: a Simonyi konf demót és a GPIO vezérlést. A két task között a kontrolleren lévő piros gombbal lehet váltani.
