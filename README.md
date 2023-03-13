# INDACT robotkar alacsony szintű programkódja

## STM32F429_StepperMotorControlling Library:
A könyvtárban a létetőmotorok használatához szükséges függvények és struktúrák szerepelnek.
- enum MovementCommands: Itt vannak felsorolva a robotkar által értelmezett parancsok.
- struct ToolPosition: A megfogó pozícióját jegyzi. Segítségével helyzeteket lehet meghatározni a térben, ahova később vissza lehet térni.
- struct StepperMotor: Egy létetőmotor állapotát, a hozzátartozó timer referenciáját és az engedélyező és irány állító GPIO lábak azonosítóját tárolja.
- fc setMotorDir(): Megvizsgálja, hogy az általunk beállítani kívánt motorirány le van-e tiltva. Ha nem, akkor beállítja azt.
- fc setAllMotorDirTowardsDesiredPos(): A jelenlegi pozícióból és a következő pozícióból kiszámítja, hogy a motorokat merre kell forgatni a cél eléréséhez. Egyben be is állítja az irányokat.
- fc (all)motorON(): Elindítja a motorokat, miközben frissíti a motorok struktúrájában az állapotukat.
- fc startMotor(): Egyben elvégzi a setMotorDir() és motorON() függvényeket. Csak akkor indítja el a motort, ha nincs letiltva az irány.
- fc stop(All)Motor(): Megállítja a motort és átírja az állapotát ennek megfelelően.
- fc controlMotor_viaGPIO(): Az argumentumban megadott két gombnak megfelelően kapcsolja be a motort. Az egyik a pozitív irányba, a másik negatív irányba indítja. Ha     mindkettő nyomva van, akkor nem indul el a motort.

## Karcsi STM32CubeIDE projekt:
- A projekt egy STM32F429I-DISC1 Discovery board-ra készült. Az .ioc fájlban hozzáadtam a projekthez a Middleware/FREERTOS menüben az operációs rendszert.
  A FreeRTOS (free real-time operating system) egy olyan mikrokontrollerekre készült OP rendszer, ami megkönnyíti a feladatok (task) ütemezését, ezzel
  meggyorsítja a fejlesztési időt.
  - LED Indicating Task:
    Ha hiba adódik valahol a programban, a task a kártyán lévő piros LEDet kezdi villogtatni, ezzel jelezve a gondot. Ha nincs hiba, egy zöld led villog lassan.
    Jelentősége debugoláskor van.
  - Update limit_switches Task: A karon van 6 darab végálláskapcsoló, aminek mindenkori állapotát egy tömbben tárolom. A task azért felel, hogy ezeket az állapotokat       folyamatosan frissítse.
  - GPIO Controlling Task: A task két részből áll: az első egyszer fut le a task indulásakor, a második fele egy végtelen hurokban ismétlődik. Az első részben             inicializálom a motorokat és elvégzem az úgynevezett homing-olást. Ennek lényege, hogy a kar minden tengelyét kiviszem a nulla állapotba és ebben a helyzetben         frissítem a pozíciót számon tartó változót. Ezek után ha mozgatom a kart már tudni fogom, hogy a koordináta-rendszerben pontosan hol járok. A végtelen hurokban a       controlMotor_viaGPIO() függvény meghívásával motoronként 2 gombbal irányítom a kart.
  - Wifi Controlling Task: A karhoz van egy weboldal, amin keresztül utasításokat lehet kiadni. Az MCU és a böngésző között egy wifi alapú kommunikáció biztosítja a       kapcsolatot. A wifi modul UART-on keresztül küldi az utasításokat az MCU-nak. A programban az utasítások egy, a freeRTOS-ban implementált, úgynevezett Queue-ba         kerülnek (gyakorlatilag egy stack struktúra), amiből az utasítások kivételével lehet azokat végrehajtani.
    A task ezt a működést implementálja: Induláskor lefut a motor inicializáló és homingoló rész, majd a végtelen ciklusban a queue-ból várja és hajtja végre az           utasításokat. Adott időpillanatban csak egy utasítást képes futtatni! A weboldal és a kommunikáció másik oldali implementációjához nézd meg az ahhoz tartozó           [branch](https://github.com/legokor/INDACT_RobotArm/tree/wifi)-et.

- Mivel jelenleg nincsenek enkóderek felszerelve a karra, a pozíció meghatározására timer megszakításokat használok: A léptetőmotorokat PWM jellel kell szabályozni,
  amiket timerekkel állítok elő. Minden egyes PWM jel felfutó élre a timer megszakítást kér. A megszakítási rutinban az adott timerhez tartozó változót növelem
  vagy csökkentem annak függvényében, hogy a motor merre megy. A megoldás nem túl ideális, de jelenleg teljesen működőképes.
- A végálláskapcsolók is megszakítást generálnak minden kapcsoláskor. A GPIO megszakítási rutinban az EXTI line-nak megfelelően letiltom az adott motor megfelelő         iránybeli mozgását. Erre (érthető módon) azért van szükség, hogy semmilyen program vagy felhasználó ne tehessen kárt a karban azzal, hogy a megengedettnél tovább       hajtja a motorokat.
