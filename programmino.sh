#!/bin/bash
#################################################FUNZIONI#####################################################################
export WHITE="\e[1;37m"
export LGRAY="\e[0;37m"
export GRAY="\e[1;30m"
export BLACK="\e[0;30m"
export RED="\e[0;31m"
export LRED="\e[1;31m"
export GREEN="\e[0;32m"
export LGREEN="\e[1;32m"
export BROWN="\e[0;33m"
export YELLOW="\e[1;33m"
export BLUE="\e[0;34m"
export LBLUE="\e[1;34m"
export PURPLE="\e[0;35m"
export PINK="\e[1;35m"
export CYAN="\e[0;36m"
export LCYAN="\e[1;36m"
export Z="\e[0m"

function visualizzatore {
	./rgbd-multikinect
	clear
}
function acquisizione {
	
	cd ~/Scrivania/tirocinio/FrameGrabber/dist/Debug/GNU-Linux-x86/
	echo -e "$LBLUE Attendere prego alcuni secondi$Z"
	echo -e "$LBLUE Grabbing...$Z"
	./framegrabber > ~/rgbdemo/build/bin/myLog/log_frameGrabber.txt
	clear
	if [ -e color1.png -a color2.png -a depth1.raw -a depth2.raw ]
	then
		echo -e "$LBLUE Acquisiti i frame dalle 2 kinect in modo corretto. Risoluzione 1280x1024$Z"
		cd ~/rgbdemo/build/bin/

		if [ -d calibration ]
		then
			rm -r calibration
		fi
		
		cd ~/rgbdemo/build/bin
		mkdir calibration
		cd ~/rgbdemo/build/bin/file-calibration/
		cp calibration-A00364A16016051A.yml calibration-B00363210002036B.yml ~/rgbdemo/build/bin/calibration
		cd ../calibration
		mkdir -p A00364A16016051A/view0000-0,669000/raw B00363210002036B/view0001-163,050003/raw
		cd ~/Scrivania/tirocinio/FrameGrabber/dist/Debug/GNU-Linux-x86
		mv color1.png depth1.raw ~/rgbdemo/build/bin/calibration/A00364A16016051A/view0000-0,669000/raw
		mv color2.png depth2.raw ~/rgbdemo/build/bin/calibration/B00363210002036B/view0001-163,050003/raw
		cd ~/rgbdemo/build/bin/calibration/A00364A16016051A/view0000-0,669000/raw
		mv color1.png color.png
		mv depth1.raw depth.raw
		cd ~/rgbdemo/build/bin/calibration/B00363210002036B/view0001-163,050003/raw
		mv color2.png color.png
		mv depth2.raw depth.raw
		continue;
	else
		echo -e "$RED Non sono riuscito ad acqiusire correttamente i frame dalle 2 kinect in modo corretto...$Z"
		echo -e "$RED->Richiama ancora la funzione che acquisisce i frame!$Z"
		pkill Xn #faccio terminare qualunque processo che puo interferire con la kinect
		continue;
	fi
		
}

function calibrazione_intrinseci {

	echo -e "$LBLUE Attendere prego alcuni secondi che avvenga la clibrazione degli intrinseci di entrambe le kinect$Z"
	echo -e "$LBLUE Calibration...$Z"
	cd ~/rgbdemo/build/bin/
	./calibrate-openni-intrinsics calibration/A00364A16016051A/ calibration/calibration-A00364A16016051A.yml --pattern-size 0.024 2> myLog/log_intrinseci1.txt
	errore1=`tail -1 myLog/log_intrinseci1.txt | cut -d ":" -f3 | cut -d " " -f2 | cut -d "\"" -f1`
	
	./calibrate-openni-intrinsics calibration/B00363210002036B/ calibration/calibration-B00363210002036B.yml --pattern-size 0.024 2> myLog/log_intrinseci2.txt
	errore2=`tail -1 myLog/log_intrinseci2.txt| cut -d ":" -f3 | cut -d " " -f2 | cut -d "\"" -f1`
	ok=0.8

	test=`echo "$errore1 <= $ok" | bc` #0falso 1 vero
	test2=`echo "$errore2 <= $ok" | bc` #0falso 1 vero

	#test kinect 1
	echo -e "$YELLOW Errore medio sulla calibrazione degli intrinseci Kinect1: $errore1 $Z"
	if [ $test -eq 1 ]
	then
		echo -e "$RED Occorre rifare il grab dei frame della kinect 1. Spostare quindi la kinect 1 $Z"
	else
		echo -e "$YELLOW Ottima acquisizione dei frame-> Calibrazione intrinseci kinect 1 riuscita!$Z"
		echo "											"
	fi

	#test kinect 2
	echo -e "$YELLOW Errore medio sulla calibrazione degli intrinseci Kinect2: $errore2 $Z"
	if [ $test2 -eq 1 ]
	then
		echo -e "$RED Occorre rifare il grab dei frame della kinect 2. Spostare quindi la kinect 2$Z"
	else
		echo -e "$YELLOW Ottima acquisizione dei frame-> Calibrazione intrinseci kinect 2 riuscita!$Z"
	fi
	continue;
}

function calibrazione_estrinseci {
	
	echo -e "$LBLUE Attendere prego alcuni secondi che avvenga la calibrazione degli estrinseci$Z"
	echo -e "$LBLUE Calibration...$Z"
	cd ~/rgbdemo/build/bin/
	./calibrate-multiple-kinects calibration/A00364A16016051A/ calibration/B00363210002036B/ calibration/calibration-A00364A16016051A.yml calibration/calibration-B00363210002036B.yml --pattern-size 0.024 >& myLog/log_estrinseci.txt
	errore_ext=`fgrep -i "Average" myLog/log_estrinseci.txt | cut -d ":" -f2 | cut -d " " -f2`
	if [ -n "$errore_ext" ]
	then #se erorore ext non e vuoto
		echo -e "$YELLOW Errore medio sulla calibrazione degli estrinseci: $errore_ext $Z";
		soglia_accettate=0.5
		test2=`echo "$errore_ext <= $soglia_accettate" | bc`; 

		if [ $test2 -eq 0 ]; then #0falso 1 vero
			echo -e "$RED Occorre rifare il grab dei frame della kinect perche errore troppo GRANDE!$Z";
		else
			echo -e "$YELLOW Ottima acquisizione dei frame-> Calibrazione estrinseci riuscita!$Z";
			#passo il file di calibrazione degli estrinseci al programma che mi permette di fare icp
			cp ~/rgbdemo/build/bin/calibration_multikinect.yml ~/Scrivania/tirocinio/drunken-ironman-revenge/
		fi;
	else
		echo -e "$RED Occorre rifare il grab dei frame della kinect->Angoli scacchiera non trovati$Z"
	fi
	continue;
}

function icp {
	echo -e "$RED HELP COMANDI:$Z "
	echo "											"
	echo -e "$RED Premi s per passare dalla modalità acquisizione simultaneo o sequenziale$Z"  
	echo -e "$RED Premi a per ottenere la nuvola di punti$Z"
	echo -e "$RED Premi c per eliminare le nuvola di punti$Z"
	echo -e "$RED Premi n per applicare l' ICP$Z"
	echo -e "$RED Premi q per uscire dal programma$Z"
	echo -e "$RED __________________________________________$Z"
	echo "											"
	cd ~/Scrivania/tirocinio/drunken-ironman-revenge/
	~/Scrivania/tirocinio/drunken-ironman-revenge/dist/Debug/GNU-Linux-x86/./drunken-ironman-revenge
	clear;	
	continue;
}
###############################################MAIN#######################################################################################
clear
echo -e "$RED *** $Z $YELLOW Benvenuti nel centro calibrazione Kinect$Z $RED *** $Z"
for((;;))
do
	echo "											"
	echo -e "$LGREEN__________________________________________$Z"
	echo -e "$LGREEN FAI LA TUA SCELTA.$Z"
	echo -e "$LGREEN 1)Programma di visualizzazione scena.$Z"
	echo -e "$LGREEN 2)Fai partire il programma di greb dei frame.$Z"
	echo -e "$LGREEN 3)Calibrazione intrinseci.$Z"
	echo -e "$LGREEN 4)Calibrazione estrinseci.$Z"
	echo -e "$LGREEN 5)Fai partire il programma che esegue ICP di 2 nuvole di punti.$Z"
	echo -e "$LGREEN 6)Esci.$Z"
	echo -n -e "$LGREEN Digita il numero corrispondente: $Z"

	read scelta

	case $scelta in	
		1)clear
		pkill Xn #faccio terminare qualunque processo che puo interferire con la kinect
		visualizzatore;;
		2)clear
		pkill Xn #faccio terminare qualunque processo che puo interferire con la kinect
		acquisizione;;
		3)clear
		calibrazione_intrinseci;;
		4)clear
		calibrazione_estrinseci;;
		5)clear
		pkill Xn #faccio terminare qualunque processo che puo interferire con la kinect
		icp;;
		6)echo -e "$YELLOW Ciao hai finito$Z"
		exit 0;;
		*)
		echo -e "$RED $scelta operazione non valida!$Z"
		continue;;
	esac
done

