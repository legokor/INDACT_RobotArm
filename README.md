# INDACT robotkar programkódja

## Karcsi STM32CubeIDE projekt:
A projekt egy STM32F429I-DISC1 Discovery board-ra készült. Az .ioc fájlban hozzáadtam a projekthez a Middleware/FREERTOS menüben az operációs rendszert.
  A FreeRTOS (free real-time operating system) egy olyan mikrokontrollerekre készült OP rendszer, ami megkönnyíti a feladatok (task) ütemezését, ezzel
  meggyorsítja a fejlesztési időt.
  - **_LED Indicating Task_**: Ha hiba adódik valahol a programban, a task a kártyán lévő piros LEDet kezdi villogtatni, ezzel jelezve a gondot. Ha nincs hiba, egy zöld led     villog lassan. Jelentősége debugoláskor van.
  - **_GPIO Controlling Task_**: A task két részből áll: az első egyszer fut le a task indulásakor, a második fele egy végtelen hurokban ismétlődik. Az első részben       inicializálom a motorokat és elvégzem az úgynevezett homing-olást. Ennek lényege, hogy a kar minden tengelyét kiviszem a nulla állapotba és ebben a helyzetben         frissítem a pozíciót számon tartó változót. Ezek után ha mozgatom a kart már tudni fogom, hogy a koordináta-rendszerben pontosan hol járok. A végtelen hurokban a       controlMotor_viaGPIO() függvény meghívásával motoronként 2 gombbal irányítom a kart.
  - **_Simonyi Konferencia Demó Task_**: A konfra készült egy látványosságnak szánt task. Ebben a motor egy előre megírt pályát követ végtelen ciklusban. Igyekeztem       úgy megírni a pályát, hogy ha kitesszük standolni, akkor se a bámészkodó emberekbe, se a standolókba ne akadjon bele a robot. Mivel a task folyamatosan járatja a       motorokat és még nincs használatban a normális táp, ezért figyelni kell a tápegység melegedésére. Nem szerencsés hosszú időn keresztül járatni a kart.


Mivel jelenleg nincsenek enkóderek felszerelve a karra, a pozíció meghatározására timer megszakításokat használok: A léptetőmotorokat PWM jellel kell szabályozni,
  amiket timerekkel állítok elő. Minden egyes PWM jel felfutó élre a timer megszakítást kér. A megszakítási rutinban az adott timerhez tartozó változót növelem
  vagy csökkentem annak függvényében, hogy a motor merre megy.
  A végálláskapcsolók is megszakítást generálnak minden kapcsoláskor. A GPIO megszakítási rutinban az EXTI line-nak megfelelően korlátozom az adott motor mozgását.

## XX. Simonyi Konferencia:
Megírtam a kódot amivel kivisszük standolni Karcsit. A program két taskot foglal magába: a Simonyi konf demót és a GPIO vezérlést. A két task között a kontrolleren lévő piros gombbal lehet váltani.
