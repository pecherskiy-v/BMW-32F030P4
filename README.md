#Управления Люком BMW E34
#### на базе микроконтроллера stm32f030f4 (младший из семейства)
#### драйвер двигателя VNH2SP30

 назначение ног именованое в CubeMX
 
 в нем же можно ознакомиться с визуализацией настрокйи мк 


# BMW-32F030P4
req: git@github.com:pecherskiy-v/monstermoto-library-lite.git

нужно склонировать в проект, в директорию "Core\VNH2SP30\"

это библиотека работы с драйвером VNH2SP30

https://github.com/pecherskiy-v/monstermoto-library-lite


#### Video
- https://youtu.be/Y4bCqOt50oQ движение тросов
- https://youtu.be/3sa6kR8XdqA тестирование на стоде часть первая
- https://youtu.be/2PSHWo_YOpY


##PIN OUT
| PIN |   NAME   |  ALIAS  |   COLOR  |
|-----|----------|---------|----------|
| PA0 |концевик 1|endPointA|жёлт.-фио |
| PA1 |концевик 2|endPointB|жёлт.-чёрн|

##SEE
https://istarik.ru/blog/stm32/142.html