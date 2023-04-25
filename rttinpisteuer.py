from threading import Thread,Event
from serial import Serial
import time
import cv2 as cv
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
from numpy.linalg import norm
import os

#Datei für die Steuerung des Roboters
#Der Roboter ist in der Lage, zu fahren unnd auf der Fahrt eine mit einem ArUcO-Tag
#versehene Probe aufzunehmen und wieder loszulassen 
#Der roboter nutzt ein kartesisches Koordinatensystem für sich selbst 
#(0,0 ist der Drehpunkt des Roboters bei einer rotationsbewegung)
#und eines, welches 0,0 als die Startposition des Roboters betrachtet
#Das hier ist der Code auf dem Raspberry Pi. Der Pi kümmert sich um die Erinnerung der position,
#die Fahrprozedur und die visuelle erkennung von Objekten. 
#Um den Greifarm und die Fahrbasis kümmert sich der Arduino

pos=[0,0]#aktuelle position eines arucotags
Winkel=0
MP_arm=np.array([90,84])#Mittelpunkt des Greifarmes des Roboters

aufgehoben=True

robot_pos=np.array([0.0,0.0])#Position des Roboters im Feld
#Folgende Notizen sind für die Beschreibung vom Roboter wichtig.
#Der Roboter bestimmt Geometrisch die Lage einer Probe durch die Pixelposition des Mittelpunkts des ArUcO Codes
#70 pixel=90 mm
#250 pixel 250 mm
#16° bei 90
#38° bei 250
#1 roboter vorwärts
#2 roboter nach rechts
#3 roboter nach links
#4 roboter zurück
#5 Greifarm runter
#6 Greifarm hoch
#7 Greifarm rotieren
#8 Pumpe an/aus
#9 unterbrechen

def notieren(st):
    """Berechnung der aktuellen Position des Roboters, wird bei jedem Fahrbefehl aufgerufen
    st:(String): der befehl an den arduinpo als string. Wenn befehl unterbrochen, sind das hier die tatsächlich gefahrenen schritte
    """
    global robot_pos,Winkel
    z=[int(i) for i in st.split(",")]
    print(z[0],z[1])
    
    if z[0] in [1,4]:#unsauber, der Gedanke ist, dass der Roboter vorwärtsschritte poisiv und rückwärtsschritte negativ rechnet
        robot_pos+=(2.5-z[0])/1.5*z[1]*np.array([np.sin(Winkel*np.pi/180),np.cos(Winkel*np.pi/180)])
    elif z[0] in [2,3]:
        #print("rot",)
        Winkel+=(2.5-float(z[0]))/0.5*z[1]#aktueller rotationswinkel wird abgezogen, falls sich roboter nach links dreht
    print("akt:",Winkel,robot_pos)
def abstand(lin,pt):
    return norm(np.cross(lin[0]-pt,lin[1]))/norm(lin[1])

def dercap(camera,rawCapture):
    """
    Funktion für die Kamera, wird von einem Thread aufgerufen und immer weiter ausgeführt.
    camera: Handler für den Kameraport
    rawCapture: Format für die PiCamera
    """
    global pos
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        key = cv.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        imag=image.copy()
        #arucoParams = cv.aruco.DetectorParameters_create()
        corners,ids,rejected=cv.aruco.detectMarkers(imag, dicter)
        its=np.array(ids)
        if its.size==0:
            pass
        else:
            if corners:
                ev.set()#setzt ein Unterbrechungsevent für die Kameraposition, falls ArUcO Code gefunden
                corner=np.array(corners)[0][0]
                x=np.mean(corner[:,0])
                y=np.mean(corner[:,1])
                pos=[x,y]#pos ist eine globale Variable und wird von anderen Funktionen aufgerufen. ToDo: Nutzung von Alternativen zu globalen Variablen
                #corner=np.array(corners)[0][0]
            else:
                pos=[0,0]

def warter(interrupt=False):
    """funktion zum warten auf die Rückgabe des arduino. Der Arduino gibt ein signal zurück, wenn das Kommando fertig ist

    Args:
        interrupt (bool, optional): wenn true, kann warteprozess durch kameraevent abgebrochen werden. Defaults to False.

    Returns:
        st(string): gibt das kommando zurück, das eingegeben wurde. Wenn der Arduino in seiner Fahrprozedur durch die kamera abgebrochen wurd,
        gibt er die noch fehlenden Schritte des Kommandos zurück
    """
    global aktiviert
    flagg=False
    while(ser.inWaiting()==0):
        if interrupt and not aktiviert:
            #sleep wird als warten auf kameraevent gewertet
            ev.wait(timeout=0.1)
            if ev.is_set() and not aktiviert:
                print("erkannt")
                aktiviert=True#soll nicht mehrmals hintereinander aktiviert werden
                flagg=True
                ser.write("9,0".encode("utf-8"))#stoppbefehl an den arduino

        else:
            time.sleep(0.1)
    
    k=str(ser.read(ser.inWaiting()).decode("utf-8"))
    if flagg:
        return k
def printpos():
    global pos
    print(pos)
def schreib(st,kamerainterrupt=False):
    #Schreib befehl an Arduino
    ser.write(st.encode("utf-8"))
    print("geschrieben:",st,"zeit:",time.time()-tim)

    st1=warter(kamerainterrupt)
    if st1!=None:
        schrit=int(st1.split(",")[1])
        schrit2=int(st.split(",")[1])
        res=str(schrit2-schrit)
        st=st.split(",")[0]+","+res
    print(st)
    notieren(st)#soll sich merken, wie sich der roboter bewegt hat
    #return k

def interrupt():
    ints=schreib("9,00",interrupt=True)
    return ints

angpoint=0
def gehheim():#roboter fährt an startposition zurück
    global robot_pos,Winkel
    neuwinkel=np.arctan(robot_pos[0]/robot_pos[1])
    schreib("3,"+str(int(180-neuwinkel+Winkel)))
    time.sleep(0.5)
    schreib("1,"+str(int(norm(robot_pos))))
    schreib("2,"+str(int(Winkel)))
def armposproz(pt,alpha):
    posi=900+alpha*10
    schreib("7,"+str(int(posi)))
    schreib("5,50")
    schreib("6,50")
    schreib("8,1")
    schreib("7,1900")
def abstand(lin,pt):
    return norm(np.cross(lin[0]-pt,lin[1]))/norm(lin[1])
class robot_descriptor():
    def __init__(self):
        self.MP_arm=np.array([-30,60])
        self.t1_prev=0
        self.Winkel_y_unten=16#minimaler Winkel
        self.Pixel_Winkel_y=0.1215#umwandlungsfaktor in y-richtung
        self.Pixel_Winkel_x=0.132#umwandlungsfaktor in x-richtung
        self.hoehe=310#Höhe der Kamera
        self.radius_arm=110##Radius des Greifarms
        self.MP_Robot=np.array([0,0])
        self.Servo_vorn=1500#mikrosekundenanzahl, bei der der Servo nach vorn zeigt
        self.center_x=200#pixel des Zentrums von x
        self.Servo_innen=2200#hier zeigt servo ins innere des Roboters
        self.x_rechts=-115#kartesische Koordinate, die der greifarm rechts gerade so erreichen kann
        self.x_links=55#kartesische Koordinate, die der greifarm links gerade so erreichen kann
        self.strecke_ref=10#messen
    def calc_dist(self,pt):
        #berechnet die Distanz, die der Roboter geradeaus fahren muss, damit der marker 
        #durch den Greifarm erreichen muss
        lin1=np.array([pt,[0,1]])
        if(norm(pt-self.MP_arm)<self.radius_arm):
            return 0,0
        lin1[1]=lin1[1]/norm(lin1[1])
        ab=abstand(lin1,self.MP_arm)
        distgen=norm(self.MP_arm-lin1[0])
        if(ab<self.radius_arm):
            alp=np.arcsin(ab/self.radius_arm)
            bet=np.arcsin(self.radius_arm*np.cos(alp)/distgen)
            sign=1
        if (np.dot(lin1[1],self.MP_arm-lin1[0]))<0:
            sign=-1
            dister=sign*(distgen*np.cos(bet)-self.radius_arm*np.cos(alp))
            p2=lin1[0]+lin1[1]*dister
            return norm(lin1[0]-p2)
        return 0,0
    def pos_in_Winkel(self,pt):
        #rechnet pixelposition in Kamerawinkel um
        return np.array([(self.Pixel_Winkel_x)*(pt[0]-self.center_x),self.Winkel_y_unten+self.Pixel_Winkel_y*(pt[1]-71)])
    def Winkel_zu_pos(self,Wink):
        #rechnet Kamerawinkel in KArtesische Koordinate um
        y=self.hoehe*np.tan(Wink[1]*np.pi/180)
        x=np.sqrt(y**2+self.hoehe**2)*np.tan(Wink[0]*np.pi/180)
        pos=np.array([x,y])
        return pos
    def calc_dist(self,pt):
        """"
        kalkuliert den Abstand eines Punktes zum armkreis
        input:array(float,2) position x,y als array
        returns: distanz punkt zum rand des armkreises
        """
        lin1=np.array([pt,[0,1]])
        if(norm(pt-self.MP_arm)<self.radius_arm):
            return 0
        lin1[1]=lin1[1]/norm(lin1[1])
        ab=abstand(lin1,self.MP_arm)
        #distgen=norm(self.MP_arm-lin1[0])
        if(ab<self.radius_arm):
            punktkreis=np.array([pt[0],
                self.MP_arm[1]+np.sqrt(self.radius_arm**2-(pt[0]-self.MP_arm[0])**2)])
            dister=pt[1]-punktkreis[1]
            #bet=np.arcsin(self.radius_arm*np.cos(alp)/distgen)
            return dister
        return 0
    def armwinkel(self,pt):
        diff=pt-self.MP_arm
        return self.Servo_vorn+np.arctan(diff[0]/diff[1])*1400/np.pi#stimmt vllt nicht
    def getposs(self,pt):
        #rechnet pixelposition direkt in kartesische position um
        return self.Winkel_zu_pos(self.pos_in_Winkel(pt))
    def prozed_anf(self,pt):
        """lässt den roboter eine bestimmte position anfahren, um eine probe aufheben zu können.
        Es werden nacheinander mehrere Befehle abgegeben, um die Probe in Kontrolle des Greifarms zu bringen

        Args:
            pt (float,2): Punkte in Kartesischen Koordinaten
        """
        if(np.all(pt==0)):
            print("nix gefunden")
            return
        befehl=[]
        if pt[0]>self.x_links or pt[0]<self.x_rechts:
            #die Probe liegt zu weit links und zu weit rechts, als dass der Greifarm durch geradeausfahren des roboters
            #an die probe rankommt
            diff=pt-self.MP_Robot
            diff_complex=diff[0]+diff[1]*1j#distanz der Probe zum Robotermittelpunkt als komplexer zeiger
            Winkel=int((-np.angle(diff_complex)+np.angle(1j))*180/np.pi)
            pt=self.MP_Robot+np.array([0,norm(diff)])#korrektur des Probenstandorts um den rotationswinkel
            befehl.append("2,"+str(Winkel))
        if norm(pt-self.MP_arm)>self.radius_arm:#Probe zu weit entfernt, Roboter muss sich nach vorne bewegen
            abstand=self.calc_dist(pt)
            steps=int((abstand+5)/self.strecke_ref)
            pt=pt-np.array([0,abstand])#korrektor des Probenpunkte um die vorwärtsbewegung
            befehl.append("1,"+str(steps))
        alpha=self.armwinkel(pt)#berechnung des Winkels der Probe gegenüber dem Greifarm
        befehl.append("7,"+str(int(alpha)))   #greifarmbewegung
        print(befehl) 
        for i in befehl:
            schreib(i)
    def finver(self):
        #fährt den Roboter geradeaus, wenn probe sichtbar, wird die geradeausfahrt unterbrochen,
        #dann fährt der roboter zurücl
        global pos,aktiviert,Winkel,robot_pos
        schreib("1,80",kamerainterrupt=True)
        if aktiviert:
            self.prozed_anf(self.getposs(pos))
        schreib("8,1")#Pumpe an
        schreib("5,102")#greifarm runter
        time.sleep(1)
        schreib("6,102")#greifarm rauf
        gehheim()
        Winkel=0
        robot_pos=np.array([0.0,0.0])
        
        schreib("5,30")#ablegen probe
        schreib("8,0")
        time.sleep(1)
        schreib("7,1200")
        schreib("7,1900")
        schreib("7,1200")
        
        #schreib("8,0")
        schreib("6,30")

        aktiviert=False#zurücksetzewn der robotereinstellungen
        ev.clear()

    def fahr(self,pt,pumpaus=True):
        if pt==[0,0]:
            return
        pos=self.getposs(pt)
        self.prozed_anf(pos)
        schreib("8,1")
        schreib("5,65")
        time.sleep(1)
        schreib("6,65")
        if pumpaus:
            schreib("8,0")
#befehle für den anfang: rttin.cos.getposs(rttin.pos)
#con=rttin.con
#con.prozed_anf(con.getposs(rttin.pos))
if __name__=="__main__":
    ser=Serial("/dev/ttyUSB0",baudrate=9600)#verbindung zum Arduino
    print("jetzt")
    ev=Event()
    tim=time.time()
    camera = PiCamera()
    camera.framerate=10
    camera.resolution = (400, 300)
    rawCapture = PiRGBArray(camera,size=(400,300))
    dicter=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100) 
    T1=Thread(target=dercap,args=(camera,rawCapture))
    T1.start()

    #passender punkt: 110,85 für median
    #prev=False
    aktiviert=False
    schreib("7,2200")       
    schreib("8,0")
    con=robot_descriptor()
    com=input("""Bitte Geben sie einen Befehl ein: \n
            w (Schritte): Vorwärts in Schritten
            a (grad): rotieren nach links in grad
            d (grad): rotieren nach rechts in grad
            s (Schritte): Rückwärts
            pos: position eines ArUcOs()(0,0) bei nicht vorhandensein
            code_anfahren : Fährt einen QrCode an, falls vorhanden
            erkunden: Fährt durch die Landschaft, bis ein code erkannt wirdl, nimmt probe audf und fährt zurück
                """)
    com=com.split(" ")
    if com[0]=="w":
        schreib("1,"+int(float(com[1])))
    elif com[0]=="a":
        schreib("2,"+int(float(com[1])))
    elif com[0]=="d":
        schreib("3,"+int(float(com[1])))
    elif com[0]=="s":
        schreib("4,"+int(float(com[1])))
    elif com[0]=="pos":
        print(con.getposs(pos))
    elif com[0]=="code_anfahren":
        con.prozed_anf(con.getposs(pos))
    elif com[0]=="erkunden":
        con.finver()


