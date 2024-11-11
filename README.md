# Semestralne_zadanie_35b

Tento projekt má za úlohu zostaviť a naprogramovať senzor prekážok. Dáta bude získavať z troch viacsegmentových laserových diaľkomerov a na základe nich navrhne odporúčaný smer pohybu. 
Tento návrh bude následne zobrazený na 7-segmentovom displeji, pričom bude využívať znaky v tabuľke nižšie.  

<div align="center">
	<table>
	  <tr> <th>Odporúčanie</th> <th>Znamenie na displeji</th> </tr>
	  <tr> <td>silne doľava</td> <td>E---</td>  </tr> 
	  <tr>  <td>slabo doľava</td> <td>E-</td> </tr>
	  <tr> <td>dopredu</td> <td>-ƎE-</td> </tr>
	  <tr> <td>prekážka v strede</td> <td>E--Ǝ</td> </tr>
	  <tr> <td>slabo doprava</td> <td>-Ǝ</td> </tr>
	  <tr> <td>silne doprava</td> <td>---Ǝ</td>  </tr>
	</table>
</div>

## Použitý hardware
- sedem-segmentový displej
- tri viac-segmentové laserové diaľkomery
- mikrokontrolér STM32F303
- podkladová doska
- káble


## Vypracovanie
- Branislav Kapusta (logika v súbore main.c - prepojenie knižníc + nastavenie projektu)
- Samuel Gombár (knižnica na komunikáciu s diaľkomermi)
- Vendelín F. Skokan (knižnica na komunikáciu s displejom)
- Michal Zborovjan (logika použitia dát na vytvorenie návrhov)
