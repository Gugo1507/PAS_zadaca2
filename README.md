# PAS_zadaca2 – Dijkstra algoritam

Ovaj projekt sadrži python skriptu za pretraživanje i vizualizaciju rada Dijkstra algoritma pomoću ROS2 i RVIZ -a.

U nastavku se nalaze upute za uspješno pokretanje.

---

## ✅ 1. Kloniranje repozitorija

```bash
git clone https://github.com/Gugo1507/PAS_zadaca2.git 
```

---

## ✅ 2. Pokretanje map servera i vizualizacija mape u rviz -u

U terminalu pokrenuti rviz:

```bash
rviz2 
```

U drugom terminalu ući u folder u kojem je preuzeta mapa te pokrenuti: 

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=moja_karta.yaml
```
U trećem terminalu redom pokrenuti naredbe: 

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```
Nakon čega bi u rviz-u trebalo dodati By topic /map te bi karta trebala biti vidljiva. 
---

## ✅ 3. Pokretanje Dijsktra algoritma

U novom terminalu pokrenuti Dijkstra algoritam: 
```bash
python3 Dijkstra.py 
```
Potrebno je sada By topic dodati sljedeće stvari u rviz -u: 
- /goal_markers
- /explored_nodes
- /clicked_point
- /planned_path

Sljedeće je potrebno unutar Rviz -a pomoću "Publish Point" opcije odabrati na mapi početnu i ciljnu točku nakon čega počinje pretraživanje prostora te vizualizacija.
---


