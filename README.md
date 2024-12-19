# Semestralne_zadanie_35b

Cieľom tohto semestrálneho zadania je navrhnúť a naprogramovať senzor na detekciu prekážok. Dáta bude získavať z troch viacsegmentových laserových diaľkomerov a na základe nich navrhne odporúčaný smer pohybu. 
Tento návrh bude následne zobrazený na 7-segmentovom displeji, pričom bude využívať znaky v tabuľke nižšie.  

<div align="center">
	<table>
	  <tr> <th>Odporúčanie</th> <th>Znamenie na displeji</th> </tr>
	  <tr> <td>silno doľava</td> <td>E---</td>  </tr> 
	  <tr>  <td>slabo doľava</td> <td>E-</td> </tr>
	  <tr> <td>dopredu</td> <td>-ƎE-</td> </tr>
	  <tr> <td>prekážka v strede</td> <td>E--Ǝ</td> </tr>
	  <tr> <td>slabo doprava</td> <td>-Ǝ</td> </tr>
	  <tr> <td>silno doprava</td> <td>---Ǝ</td>  </tr>
	</table>
</div>

## Použitý hardware
- sedem-segmentový displej
- tri laserové diaľkomery
- mikrokontrolér STM32F303
- podkladová doska
- káble


## Vypracovanie
- Branislav Kapusta (logika v súbore main.c - prepojenie knižníc + nastavenie i2c read/write)
- Samuel Gombár (knižnica na komunikáciu s diaľkomermi)
- Vendelín F. Skokan (knižnica na komunikáciu s displejom)
- Michal Zborovjan (logika použitia dát na vytvorenie návrhov)

## Diagram

![MainLogicDiagram drawio](https://github.com/user-attachments/assets/3bac6d6b-7465-4ebd-8832-9660a513969b)

## Riešenie
Pre čítanie dát zo senzora používame knižnicu HAL, ktorá prečíta bod z daného senzora a potom danú hodnotu ukladá do premmennej, ktorú neskôr používame v rozhodovacom algoritme na určovanie požadovaného otočenia. Taktiež máme vytvorené riešenie pre spracovanie dát s použitím LL knižníc aby bolo možné dáta vypisovať na display taktiež pomocou LL knižníc, kde zapisovanie dát funguje ale čítanie dát zo senzora je problémové.

Na display sa vypisujú znaky z tabuľky vyššie, ktoré naznačujú, pod ktorým uhlom smerujú prekážky. Na display sme pripojený pomocou I2C cez LL knižnicu, ktorá komunikuje s čipom STMPE1600. Tento čip funguje ako GPIO expander, kde jeden pin má vstupný a 16 pinov je výstupných. Display je pripojený k SDA a SCL, a pomocoudaného čipu, je možné vypisovať dáta na display. Na čip je nutné posielať binárny kód, ktorý určuje, ktorý segment bude páve svietiť. 

#monkey
