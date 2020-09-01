# AGV  (AUTOMATED GUIDED VEHICLE)
PRATICA SRL – www.praticasrl.com  
Progetto: AGV - Veicolo a guida automatica per movimentazione pallet  
Progetto cofinanziato dal Fondo POR FESR 14/20 Calabria 

# TURTLEBOT3
Il primo prototipo di AGV è stato sviluppato sul Turtlebot3 Robotis (https://www.turtlebot.com)

![alt text](https://imgur.com/rx3XeKB.png)

# ESEMPIO APPLICATIVO MOVIMENTAZIONE PALLET
![alt text](https://imgur.com/5odDpaY.png)

E' sopra riportato un esempio applicativo di un AGV per la movimentazione pallet in un magazzino parzialmente indoor e parzialmente outdoor. In ambito indoor è usato il sistema di localizzazione RTLS UWB (che comprende un insieme di anchor dislocati nell'ambiente ed un tag a bordo veicolo); in ambito outdoor è usato il sistema di localizzazione GPS RTK (che comprende una base fissa con visibilità ottimale dei satelliti ed un rover a bordo veicolo). Il sistema è in grado di ricevere una coordinata di carico; portarsi a tale coordinata; effettuare il carico del pallet tramite computer vision; portarsi alla coordinata di scarico. 


# FLOW CHART OPERATIVO
![alt text](https://imgur.com/x3d0QaP.png)  
Diagramma di flusso del funzionamento di base del sistema AGV per movimentazione pallet. 
