Dieses Projekt behandelt eine Modifikation des Eurobots 2022 des Gießener Robotik-Teams M.A.M.U.T.

Der Eurobot ist ein International abgehaltener französischer Wettbewerb, in welchem Schüler und Studenten Roboter autonom Aufgaben erledigen lassen, 
ohne sich jedoch aktiv beim Spiel gegeneinander zu behindern.

Es handelt sich um einen Roboter, der in der Lage ist, autonom zu fahren und mit einem ArUcO-Marker versehene Proben aufzuheben und zu transportieren.
Auch soll der Roboter in der Lage sein, Hindernisse wie gegnerische Roboter zu erkennen und ihnen auszuweichen (Temporär defekt, funktionierte in einer früheren Version)
Der Roboter benutzt die OpenCV-Bibliothek, um ArUcO-Marker zu erkennen und berechnet die Lage dann geometrisch.
In diesem Projekt wird sich hauptsächlich der Raspberry-Pi Kamera bedient, die Unterstützung einer USB-KAmera ist nicht implementiert.

Der Arduino-code wird mit dem vorrangig für VSCode verfügbaren Addon PlatformIO geflasht, install siehe:
https://platformio.org/platformio-ide

Der Roboter arbeitet mit einem Raspberry Pi, der die Kamera bedient und mir einem Arduino Nano kommuniziert, der die restliche Aktorik behandelt

Credits:

Jonas Krajetzki: Entwurf Schaltung für die Motortreiber der Fahrbasis, der Erkennung von Hindernissen und Mithilfe bei den Platinen zur Greifarmsteuerung

Markus Blaut: Erstellung des Codes für die Fahrbasis

Lukas Schmidt: Code für die Erkennung von Hindernissen



