/* PRATICA SRL – www.praticasrl.com
Progetto: AGV - Veicolo a guida automatica per movimentazione pallet
Progetto cofinanziato dal Fondo POR FESR 14/20 Calabria
Autori:
Duardo Domenico
Scimonelli Mattia  */

#include <iostream>
#include <fstream>
#include <string>
#include <Windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>
#include <opencv2/xfeatures2d.hpp>

//Test risoluzione camera
//#define TEST_CAMERA 1;

//O uno o l'altro algoritmo
//#define SURF_ALG 1;
#define SIFT_ALG 1;

//Name spaces used 
using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;


bool test_resolution(cv::VideoCapture* camera, int width, int height, int &actualWidth, int &actualHeight)
{
	camera->set(CV_CAP_PROP_FRAME_WIDTH, width);
	camera->set(CV_CAP_PROP_FRAME_HEIGHT, height);
	width = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_WIDTH));
	height = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_HEIGHT));

	if (width != actualWidth || height != actualHeight)
	{
		actualWidth = width;
		actualHeight = height;
		return true;
	}
	else
	{
		return false;
	}
}

void query_resolutions(cv::VideoCapture* camera)
{
	int xx[] = { 160,176,192,240,320,352,352,384,480,640,640,704,720,800,800,960,1024,1280,1280,1280,1280,1600,1920 };
	int yy[] = { 120,144,144,160,240,240,288,288,360,360,480,480,480,480,600,720,768,720,800,960,1024,1200,1080 };

	int actualWidth = 1920;
	int actualHeight = 1080;

	for (int i = 0; i < 23; i++) {

		bool resoChanged = test_resolution(camera, xx[i], yy[i], actualWidth, actualHeight);
		if (resoChanged)
			std::cout << actualWidth << "x" << actualHeight << std::endl;

	}
}

int main() {

	//TEST CAMERA--------------------------------------------------------------------
#ifdef TEST_CAMERA
	VideoCapture test;
	int device = 0;             // 0 = open default camera
	int api = CAP_DSHOW;      // CAP_ANY = autodetect default API
	test.open(device + api);
	if (!test.isOpened()) return -1;
	cout << "Risoluzioni supportate:" << endl;
	query_resolutions(&test);
	test.release();
#endif // TEST_CAMERA
	//--------------------------------------------------------------------

	//PARAMETRI--------------------------------------------------------------------

	/*
	Il primo punto in SURF/SIFT consiste nel filtrare opportunamente tutti i punti rilevati dal detector
	attraverso la scelta di un valore di soglia (valore minimo del discriminante dell’Hessiana minHess),
	al di sotto del quale i pixel vengono scartati. Aumentando la soglia diminuisce il numero di punti individuati,
	i quali però sono i punti più “forti” cioè più caratteristici e originali.
	*/
	int minHess = 500;
	/*
	Soglia di corrispondenza per selezionare le corrispondenze più forti. La soglia rappresenta una percentuale della distanza da una corrispondenza perfetta.
	Due vettori feature corrispondono quando la distanza tra loro è inferiore alla soglia impostata da MatchThreshold.
	La funzione rifiuta una corrispondenza quando la distanza tra le funzioni è maggiore del valore di MatchThreshold.
	Aumentare il valore per restituire più corrispondenze.
	*/
	//float thresholdMatchingNN = 0.5; 
	int thresholdMatchingNN = 5;

	/*
	Soglia di buoni Matches, superata la quale mi disegna il rettangolo verde.
	Diminuendo tale soglia mi riconosce l'oggetto a distanze maggiori, ma con meno precisione, visto che si basa su meno punti buoni.
	*/
	int thresholdGoodMatches = 10;

	//Risoluzione schermo PC
	int screen_w = (int)GetSystemMetrics(SM_CXSCREEN);
	int screen_h = (int)GetSystemMetrics(SM_CYSCREEN);
	cout << "Risoluzione schermo= " << screen_w << "x" << screen_h << endl;

	//Parametri per modificare la risoluzione video della camera
	int WIDTH = 640; // Larghezza della finestra del video
	int HEIGHT = 480; // Lunghezza della finestra del video
	cout << "Risoluzione camera= " << WIDTH << "x" << HEIGHT << endl;

	/*
	Risoluzione in dpi. Un quadrato della griglia sul video è 100 px,
	Ho misurato sullo schermo i cm (2,4) relativi al quadrato e ho ricavato un DPI
	dalla formula risoluzione_dpi = (width_obj_px / width_obj_cm )*2.54;
	*/
	int GRIGLIA = 100; //Larghezza pixel griglia
	double risoluzione_dpi = ((GRIGLIA*2.54) / 2.4);
	cout << "DPI= " << risoluzione_dpi << endl;
	cout << "Larghezza in px griglia= " << GRIGLIA << endl;
	cout << "Larghezza in cm griglia= " << (GRIGLIA*2.54) / risoluzione_dpi << endl;

	/*
	Ho trovato una serie di misure (Distanza-Larghezza oggetto) che evidenziano come le due unità
	siano inversamente proporzionali secondo una formula distanza_oggetto_cm=160/larghezza_oggetto_cm
	*/
	int SCALA = 180;
	//--------------------------------------------------------------------


	//MAIN--------------------------------------------------------------------
#ifdef SURF_ALG
	cout << "Avvio algoritmo SURF" << endl;
#endif
#ifdef SIFT_ALG
	cout << "Avvio algoritmo SIFT" << endl;
#endif

	//Carica immagine da riconoscere
	Mat object = imread("C:/Users/Domenico/Desktop/floralys.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//Per non modificare il codice quando provo sul mio pc
	if (!object.data) {
		object = imread("C:/Users/Scimo/Desktop/book1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	}
	//In caso di errore esce
	if (!object.data) {
		cout << "Can't open image";
		return -1;
	}

	vector<KeyPoint> kpObject;  //Il vettore di input/output dei punti chiave, punti rilevati sull'immagine
	Mat desObject;   //La matrice di output dei descrittori.
	FlannBasedMatcher matcher;  //su una collezione di descrittori chiama i metodi di ricerca per trovare le migliori corrispondenze. 

#ifdef SURF_ALG
	Ptr<SURF> alg = SURF::create(minHess);
#endif
#ifdef SIFT_ALG
	Ptr<SiftFeatureDetector> alg = SiftFeatureDetector::create(minHess);
#endif
	alg->detectAndCompute(object, Mat(), kpObject, desObject);  //Rileva i punti chiave e calcola i descrittori.

	// Inizializza video
	VideoCapture cap;
	int deviceID = 0;             // 0 = open default camera
	int apiID = CAP_DSHOW;      // CAP_ANY = autodetect default API; CAP_DSHOW = ok bordi 
	cap.open(deviceID + apiID);
	if (!cap.isOpened()) return -1;

	//Impostazione risoluzione video, può essere modificata in base alla videocamera che si va ad utilizzare
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	//Inizializza finestre
	namedWindow("Riconoscimento", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
	namedWindow("Camera", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);

	//Finestra settaggi
	namedWindow("Settaggi", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
	createTrackbar("Soglia M", "Settaggi", &thresholdMatchingNN, 10);
	createTrackbar("Numero M", "Settaggi", &thresholdGoodMatches, 50);
	resizeWindow("Settaggi", 400, 0);

	//Posiziona finestre
	moveWindow("Riconoscimento", 0, 0);
	moveWindow("Camera", screen_w - WIDTH - 5, screen_h - HEIGHT - 100);
	moveWindow("Settaggi", 0, screen_h - 200);

	//Capacità fps fotocamera
	cap.set(CV_CAP_PROP_FPS, 30);
	double fps = cap.get(CV_CAP_PROP_FPS);
	cout << "FPS camera usando cap.get(CV_CAP_PROP_FPS): " << fps << endl;

	//Variabili misura FPS run-time
	int num_frames = 0;
	time_t start, end;
	double seconds = 0;

	// Punti d'angolo dell'oggetto per il plottaggio della casella
	vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0);
	obj_corners[1] = cvPoint(object.cols, 0);
	obj_corners[2] = cvPoint(object.cols, object.rows);
	obj_corners[3] = cvPoint(0, object.rows);


	//Video loop
	for (;;)
	{
		//Misuro fps
		if (num_frames == 0)
			time(&start);
		if (num_frames > 120) {
			time(&end);
			seconds = difftime(end, start);
			// Calcola fotogrammi al secondo
			fps = num_frames / seconds;
			time(&start);
			num_frames = 0;
		}
		num_frames++;

		Mat frame;
		Mat image;
		cap >> frame;
		//l'algoritmo gira su image... su frame disegno quello che voglio per la visualizzazione
		cvtColor(frame, image, CV_RGB2GRAY);  //La conversione da un'immagine RGB a grigia

		// Disegna la griglia nella finestra del video
		int u = 0;
		u = HEIGHT / 2;
		while (u >= 0) {
			line(frame, cvPoint(0, u), cvPoint(WIDTH, u), cvScalar(0, 0, 250));
			u = u - GRIGLIA;
		}
		u = HEIGHT / 2;
		while (u <= HEIGHT) {
			line(frame, cvPoint(0, u), cvPoint(WIDTH, u), cvScalar(0, 0, 250));
			u = u + GRIGLIA;
		}
		u = WIDTH / 2;
		while (u >= 0) {
			line(frame, cvPoint(u, 0), cvPoint(u, HEIGHT), cvScalar(0, 0, 250));
			u = u - GRIGLIA;
		}
		u = WIDTH / 2;
		while (u <= WIDTH) {
			line(frame, cvPoint(u, 0), cvPoint(u, HEIGHT), cvScalar(0, 0, 250));
			u = u + GRIGLIA;
		}

		// Disegna un cerchio al centro del video (origine assi)
		circle(frame, cvPoint(WIDTH / 2, HEIGHT / 2), 5, cvScalar(0, 0, 250), 1, CV_AA);

		//Stampo FPS
		std::stringstream ss;
		ss << std::fixed << std::setprecision(1) << fps;
		string text = "FPS=" + ss.str();
		//string text = "FPS=" + std::to_string((int)fps);
		putText(frame, text, cvPoint(WIDTH - 125, HEIGHT - 10), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 0), 1, CV_AA);

		Mat des_image, img_matches, H;
		vector<KeyPoint> kp_image;
		vector<vector<DMatch > > matches;  //vettore di Classe per la corrispondenza dei descrittori dei punti chiave: indice del descrittore della query, indice del descrittore del oggetto, indice dell'immagine del oggetto e distanza tra descrittori.
		vector<DMatch > good_matches;
		vector<Point2f> obj;
		vector<Point2f> scene;
		vector<Point2f> scene_corners(4);

		alg->detectAndCompute(image, Mat(), kp_image, des_image);

		//Se le righe della matrice dei descrittori dell'immagine e dell'oggetto sono maggiori o uguali a 2
		if (des_image.rows >= 2 && desObject.rows >= 2)
		{
			matcher.knnMatch(desObject, des_image, matches, 2); //trova le migliori corrispondenze, 2 per ogni descrittore da un set di query.
			for (int i = 0; i < min(des_image.rows - 1, (int)matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
			{
				float thresholdMatching = (float)thresholdMatchingNN / 10;
				//cout << "thresholdMatching: " << thresholdMatching << endl;
				if ((matches[i][0].distance < thresholdMatching * (matches[i][1].distance)) && ((int)matches[i].size() <= 2 && (int)matches[i].size() > 0))
				{
					good_matches.push_back(matches[i][0]);  //Aggiunge il valore dell'elemento specificato alla fine del vettore.
				}
			}

			// Disegna solo matches "buone"
			drawMatches(object, kpObject, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			//drawMatches(object, kpObject, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			//drawMatches(object, kpObject, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::DEFAULT);

			//Controllo per slider
			if (thresholdGoodMatches == 0)
				thresholdGoodMatches = 1;

			if (good_matches.size() >= thresholdGoodMatches)
			{

				// Mostra che l'oggetto è stato trovato
				for (unsigned int i = 0; i < good_matches.size(); i++)
				{
					//Get the keypoints from the good matches // Prendi i punti chiave dalle buone Matches
					obj.push_back(kpObject[good_matches[i].queryIdx].pt);
					scene.push_back(kp_image[good_matches[i].trainIdx].pt);
				}

				//Trova una trasformazione prospettica tra due piani.
				H = findHomography(obj, scene, CV_RANSAC);
				//Se ci sono punti in comune
				if (!H.empty())
				{
					perspectiveTransform(obj_corners, scene_corners, H);  //Calcola una trasformazione prospettica da quattro coppie dei punti corrispondenti.

					// Disegna linee tra gli angoli (l'oggetto mappato nell'immagine scena)
					line(frame, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4);
					line(frame, scene_corners[1], scene_corners[2], Scalar(0, 255, 0), 4);
					line(frame, scene_corners[2], scene_corners[3], Scalar(0, 255, 0), 4);
					line(frame, scene_corners[3], scene_corners[0], Scalar(0, 255, 0), 4);

					Point2f cen(0, 0); //Coordinate centro oggetto riconosciuto

									   /*
									   //Coordinate del punto centrale Metodo 1
									   for (size_t i = 0; i < scene.size(); i++)
									   {
									   cen.x += scene_corners[i].x;
									   cen.y += scene_corners[i].y;
									   }
									   cen.x /= scene_corners.size();
									   cen.y /= scene_corners.size();
									   /*

									   /*
									   //Coordinate del punto centrale Metodo 2
									   Point2f pmin(1000000, 1000000);
									   Point2f pmax(0, 0);
									   for (size_t i = 0; i < scene.size(); i++)
									   {
									   if (scene_corners[i].x < pmin.x) pmin.x = scene_corners[i].x;
									   if (scene_corners[i].y < pmin.y) pmin.y = scene_corners[i].y;
									   if (scene_corners[i].x > pmax.x) pmax.x = scene_corners[i].x;
									   if (scene_corners[i].y > pmax.y) pmax.y = scene_corners[i].y;
									   }
									   cen.x = (pmax.x - pmin.x) / 2;
									   cen.y = (pmax.y - pmin.y) / 2;
									   */

					//Coordinate del punto centrale Metodo Duardo
					cen.x = ((scene_corners[0].x) + (scene_corners[1].x) + (scene_corners[3].x) - (scene_corners[0]).x) / 2;
					cen.y = ((scene_corners[0].y) + (scene_corners[1].y) + (scene_corners[3].y) - (scene_corners[0]).y) / 2;

					//Disegno pallino centro immagine riconosciuta
					circle(frame, cen, 4, Scalar(0, 255, 0), 3, CV_AA);

					//Mostra che l'oggetto è stato trovato, 
					double temp = 0;
					string text = "";
					//Calcolo X in cm
					temp = ((cen.x - WIDTH / 2)*2.54 / risoluzione_dpi);
					ss.str("");
					ss.clear();
					ss << std::fixed << std::setprecision(1) << temp;
					text = "X=" + ss.str();
					//Calcolo Y in cm
					temp = ((cen.y - HEIGHT / 2)*2.54 / risoluzione_dpi);
					ss.str("");
					ss.clear();
					ss << std::fixed << std::setprecision(1) << temp;
					text = text + " Y=" + ss.str();
					//Calcolo distanza
					//Ricavo la larghezza dell'oggetto mappato in px.
					temp = ((scene_corners[1].x - scene_corners[0].x) + (scene_corners[2].x - scene_corners[3].x)) / 2;
					//Dalla larghezza in px calcolo larghezza in cm 
					temp = temp * 2.54 / risoluzione_dpi;
					//Dalla larghezza in cm alla distanza
					temp = SCALA / temp;
					ss.str("");
					ss.clear();
					ss << std::fixed << std::setprecision(1) << temp;
					text = text + " D=" + ss.str() + " [cm]";

					putText(frame, text, cvPoint(5, 20), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 0), 1, CV_AA);

				}
			}
			else
			{

			}
		}

		//Show detected matches
		if (!img_matches.empty())
			imshow("Riconoscimento", img_matches);
		imshow("Camera", frame);

		//Key for exit,
		if (waitKey(1) >= 0) break;

	}

	return 0;
}