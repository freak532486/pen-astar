# Alternativrouten durch Penalty-Methode
## Beschreibung
Dieses Programm berechnet Alternativrouten auf stark zusammenhängenden Straßengraphen mithilfe der Penalty-Methode zwischen zwei Knoten *s* und *t*. Es werden Graphen im *DIMACS*-Format benötigt. Ein *DIMACS*-Graph besteht aus drei Vektoren

 - head
 - first_out
 - weight
 
 Alle drei Vektoren bestehen aus 4-Byte-Integerwerten. Die Vektoren head und first_out definieren eine Adjazenzliste, wobei first_out die Kanten beschreibt und head die Knoten. weight hat dieselbe Größe wie first_out und beschreibt die Kantengewichte.  

Weiterhin wird eine *Contraction Hierarchy* benötigt. Diese kann mithilfe von *RoutingKit* generiert werden. Die Ordnerstruktur eines Graphen muss wie folgt aussehen:

- /first_out
- /weight
- /head
- /ch/first_out
- /ch/head
- /ch/weight

Alle anderen Dateien werden ignoriert. Das Programm prüft **nicht** die Korrektheit der Graphen. Je nach Betriebssystem ist Groß- und Kleinschreibung wichtig (auf Windows nicht, auf *nix ja).
## Kompilierung
Das Programm lässt sich mit g++ auf Windows und Linux kompilieren. Es wegen Multithreading mit der gcc-Flag `pthread` kompiliert werden. Weiterhin wird der Optimierungsmodus `O3` für beste Laufzeiten empfohlen. Es werden die freien header-only-Bibliotheken [*cxxopts*](https://github.com/jarro2783/cxxopts) und [*SpatiumLib*](https://github.com/martijnkoopman/SpatiumLib) benötigt. Diese müssen mit `-I [Pfad-Zu-Bibliotheken]` eingebunden werden. Es wird der C++17-Standard benötigt. Unten steht das gesamte Kompilierungskommando. *cxxopts* wird unter MIT-Lizenz genutzt, *SpatiumLib* unter GPL3-Lizenz.

`g++ -O3 -I [Pfad-Zu-Bibliotheken] --std=c++17 -pthread main.cpp -o penalty`

Eine Kompilierung mit MSVC ist möglich.

## Benutzung

	penalty [MODE] [OPTIONS]

Das Programm hat zwei Modi:

- generate: Generiert Quell- und Zielvektoren zum Testen
- run: Lässt die Penalty-Methode auf Quell- und Zielvektoren laufen

Für beide Modi gibt es unterschiedliche Kommandozeilenparameter. Im folgenden sei `N` eine nichtnegative ganze Zahl, `F` eine Kommazahl und `S` eine Zeichenkette:

**generate**

generate-vectors hat wieder zwei Modi: Dijkstra-Rank-Zielknoten und zufällige Knoten. Um Dijkstra-Rank-Zielknoten zu generieren, nutzt man `penalty generate rank [OPTIONS]`. Um zufällige Vektoren zu generieren nutzt man `penalty generate random [OPTIONS]`. Es werden die Vektoren `source` und `target` im Zielordner gespeichert. Für den Dijkstra-Rank-Modus wird zusätzlich der Vektor `rank` gespeichert, der einen Start-Ziel-Paar seinen Rang zuweist.

- `-i S` / `--input S`: Setzt den Pfad zum Graphen auf `S`
- `-o S` / `--output S`: Setzt den Ausgabepfad auf `S`
- `-s N` / `--source N`: Setzt Quellknoten für Dijkstra-Rank-Vektoren auf `N`
- `--source-vector S`: Lädt Quellknotenvektor aus Pfad `S`, überschreibt `--source`
- `--limit N`: Anzahl der zu generierenden Zielknoten bei `random`-Modus wird auf `N` gesetzt
- `--min-rank N`: Minimaler zu generierender Dijkstra-Rank wird auf `N` gesetzt.

**run**
**run** hat keine weiteren Modi. Es wird für `run` mindestens ein Graph, ein Ausgabeordner, ein Quellknoten und ein Zielknoten benötigt. Benutzung: `penalty run [OPTIONS]`.

- `-i S` / `--input S`: Setzt den Pfad zum Graphen auf `S`
- `-o S` / `--output S`: Setzt den Ausgabepfad auf `S`
- `-s N` / `--source N`: Setzt Quellknoten auf `N` (zu nutzen mit `-t`)
- `-t N` / `--target N`: Setzt Zielknoten auf `N` (zu nutzen mit `-s`)
- `-q` / `--quality`: Wenn -q gesetzt ist, werden Qualitätsfaktoren für die gefundenen Pfade berechnet. Dies hat starke Laufzeitkosten.
- `--source-vector S`: Setzt Pfad zum Quellknotenvektor auf `S`, überschreibt `-s`
- `--target-vector S`: Setzt Pfad zum Zielknotenvektor auf `S`, überschreibt `-t`
- `--rank-vector S`: Setzt Pfad zum Vektor, der Zielknoten einen Dijkstra-Rang zuweist, auf `S`.
- `--source-limit N`: Limitiert Anzahl der Quellknoten vom Quellknotenvektor auf `N`
- `--draw-images`: Zeichnet Bilder der gefundenen Pfade im PPM-Format. Benötigt einen `latitude` und `longitude`-Vektor im Graphordner
- `min-dijkstra-rank N`: Setzt den minimalen Dijkstra-Rank, der berechnet werden soll. Dijkstra-Ranks geringer als `2^N` werden übersprungen.
- `--alpha F`: Setzt den Alphawert der Penaltymethode auf `F` (Siehe Arbeit)
- `--eps F`: Setzt den Epsilonwert der Penaltymethode auf `F` (Siehe Arbeit)
- `pen F`: Setzt den Penalty-Faktor der Penaltymethode auf `F` (Siehe Arbeit)
- `logname S`: Setzt den Namen der Logdatei auf `S.json`

