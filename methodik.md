# Methodik

## Vorgehensweise
- Erstellen des Enviroments mithilfe des gegebenen PyBullet Env
- Testung der grundelegenden Funktionalitäten des Env und Roboters
- Recherche und Vergleich der verschiedenen Reinforcement Learning Ansätze der Vorlesung → Eintscheidung q-Learning zu verwenden
- Definition der Gridworld und Abbildung des Env in einer Map
- Aufbauen der Q-Learning-Funktionalität
- Verknüpfung zwischen Bahnplanung aus dem q-Learning und der Roboterbewegung
- Simulation und Test der Roboterbewegungen
- Erstellen von Abbildungen der Bahnplanung und erstellen der Grafiken für die Dokumentation
- Verbesserung und Anpassung der Umgebung

## Probleme
- Eigenschaften der Objekte (Dimension, Masse, Reibung) führten zu Komplikationen mit dem Greifen und Schieben
- Bahnplanung durch zu niedrigen Step-Count vorzeitig beendet
- Zu fein aufgelöstes Grid der Map resultierte in disproportional langen Trainingsdauer bei kleiner Verbesserung des Lösungswegs
- Verlassen des Workspace
- Ruckeln des Greifers ohne Fortbewegung des Objekts
- Kollision und folgendes Kratzen des Greifers auf der Arbeitsplatte
- Objekte und Hindernisse spawnen außerhalb der Map, übereinander und ineinander

## Umgesetzte Verbesserungen
- Erhöhung des Parameters Step-Count bei jeder Iteration
- Verminderung der Grid Auflösung (kürzere Trainignsdauer)
- Hindernis-Rahmen und Einschränken des Bewegunsraumsums des Roboters um die möglichen Bewegungen des Roboters einzuschränken
- Offset des Greifers von der Arbeitsplatte
- Aktivieren der Physik-Engine und implenentieren einer Memory-Funktion, wo bereits Objekte sind

## Mögliche Verbesserungen und Erweiterungen
- Rotation der Objekte und des Greifers ermöglichen
- Step-Count dynamisch an den Abstand von der Target Area anpassen
- Q-Learning durch weitere Algorithmen erweitern
- Weiter Formen für Manipulationsobjekte und Hindernisse
- Implementierung einer Objekterkennunnung mit der integrierten virtuellen Kamera
