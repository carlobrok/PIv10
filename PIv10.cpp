#include <iostream>
#include "opencv2/opencv.hpp"
#include "boost/asio.hpp"
#include <thread>
#include <chrono>
#include "VideoServer.h"
#include "CameraCapture.h"
#include "wiringPi.h"
#include "wiringSerial.h"


using namespace cv;

int pdif(int m) {
	if(m < 0)
		return m*-1;
	else
		return m;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void newLine(int fd) {
	serialPuts(fd, "\n");
}

bool rightLenght(int n) {
	if(n >= 0 && n < 1000) {
		return true;
	} else {
		return false;
	}
}

int main() {
	//Kamera cap(0) als quelle
	CameraCapture cam(0);

	cam.set(cv::CAP_PROP_FPS, 30);
	cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

	// Serielle Schnittstelle Öfnnen
	int fd = serialOpen("/dev/serial0", 115200);

	if(fd < 0){
		std::cout << "Fehler beim Öffnen der seriellen Schnittstelle" << std::endl;
		return 1;
	}

	// GPIO öffnen
	if (wiringPiSetup() == -1) {
		std::cout << "Fehler beim wiringPi Setup!" << std::endl;
		return 1;
	}

	// Pins als output und reset auf 0
	pinMode(5, OUTPUT);

	digitalWrite(5, 0);

	std::cout << "pin 5 Low" << std::endl;

	//Erstelle cv::Mat für Eingabebild
	Mat img_rgb;
	Mat img_bin;
	Mat img_hsv;
	Mat img_sw;
	Mat img_binsw;
	Mat img_gminb;		 //gruen minus blau
	Mat img_gminr;		 //gruen minus rot
	Mat img_unterschied; //unterschied zwichen den beiden oberen /\ ( gminb und gminr )
	Mat img_binunt; 		 //Binärisiert auf den Unterschied
	Mat img_binhell; 		 //Binärisiert auf die Helligkeit

	Rect rois[5];

	std::vector<Mat> splitmat;  //Ein Vektor von Matritzen für Rot, Gruen und Blau
	std::vector<Mat> splithsv;  //Ein Vektor von Matritzen für den Dynamischen Threshold (splitted HSV img)

	//Erzeuge Vektor für die Konturen
	std::vector< std::vector<Point> > contg;
	std::vector< std::vector<Point> > cont1;
	std::vector< std::vector<Point> > cont2;
	std::vector< std::vector<Point> > cont3;
	std::vector< std::vector<Point> > cont4;
	std::vector< std::vector<Point> > cont5;

	bool senddata = false;
//	int parcState = 0;

	int grState = 0;
	int grDist = 0;

	unsigned long bildnr = 0;
	unsigned int thresh = 80;


	enum grPoints {
		GRNOT = 0,
		GRLEFT = 1,
		GRRIGHT = 2,
		GRBOTH = 3,
	};

	enum pStates {
		LINE = 0,
		RAMP = 1,
		BOTTLELEFT = 2,
		BOTTLERIGHT = 3,
		ENDZONE_VICTIMS = 4,
		ENDZONE_CORNER = 5,
	};

	Scalar lowGreen = Scalar(50,90, 45);
	Scalar upperGreen = Scalar(90, 255, 110);

	//Erstelle VideoServer Objekt
	VideoServer srv;

	//Füge Fenster zum Server hinzu
	srv.namedWindow("RGB");
	srv.namedWindow("Bin");
	srv.namedWindow("SW");
	srv.namedWindow("BinSW");
	srv.namedWindow("HSV");
	srv.namedWindow("Hell");

	while(!cam.read(img_rgb));

	Point mittegr;
	Point mitte1 = Point(0,img_rgb.rows/10);
	Point mitte2 = Point(0,img_rgb.rows/10);
	Point mitte3 = Point(0,img_rgb.rows/10);
	Point mitte4 = Point(0,img_rgb.rows/10);
	Point mitte5 = Point(0,img_rgb.rows/10);

	for(int i = 0; i < 5; i++) {
		rois[i] = Rect(0,img_rgb.rows/5 * (4-i),img_rgb.cols,img_rgb.rows/5);
		std::cout << i << ": "<< rois[i] << std::endl;
	}

	//Rect roi_gross = Rect(0,img_rgb.rows/5,img_rgb.cols,img_rgb.rows/5*4);

	while(1){

//		int64_t tickPreImg = cv::getTickCount();
		while(!cam.read(img_rgb)) {

		}
		//std::cout << "Bild holen: " << (getTickCount() - tickPreImg) / cv::getTickFrequency() * 1000.0 << " ms" << std::endl;


		//Wenn neue Daten vorhanden sind, einlesen
		if(serialDataAvail(fd) > 0){
			char c = serialGetchar(fd);

			if(c != '\n' && c!= '\r') {
				std::cout << c << "   ";
				if(c == 'A') {
					serialPutchar(fd, 'A');
					std::cout << "Beginning of communication." << std::endl;
					digitalWrite(5, 0);
					serialPutchar(fd, 'B');
				}
				else if(c == '0') {
					digitalWrite(5, 0);
					std::cout << "pin 5 Low" << std::endl;
					senddata = true;
				}
			} else {
				std::cout << std::endl;
			}
		}

		if(senddata) {

			int64_t t1 = cv::getTickCount();  // Anfangs tickcount

			bildnr++;

			std::cout << "==== NEXT IMAGE ==== Bild Nr: " << bildnr << std::endl;
			GaussianBlur(img_rgb, img_rgb, Size(7,7),0,0);

			split(img_rgb,splitmat);
			img_gminb = abs(splitmat[1]-splitmat[0]);				  // gruen - blau
			img_gminr = abs(splitmat[1]-splitmat[2]);				  // gruen - rot
			img_unterschied = abs(img_gminb-img_gminr);				  // Unterschied ausrechenen; klein=schwarz oder weiß
			inRange(img_unterschied, Scalar(0), Scalar(10), img_binunt);

			cvtColor(img_rgb, img_hsv, COLOR_BGR2HSV);

			//Binarisierung mit dynamischem Schwellwert, der mit thrVal übergeben wird
			inRange(img_hsv, lowGreen, upperGreen, img_bin);
			inRange(img_hsv, Scalar(0, 0, 0), Scalar(180, 255, thresh), img_binhell); //letzet Wert ist Helligkeit

			bitwise_and(img_binunt,img_binhell,img_binsw);

			Mat result = img_binsw.clone();
			result = Scalar(0);
			std::vector<std::vector<Point> >contours;
			std::vector<Vec4i>hierarchy;
			int savedContour = -1;
			double maxArea = 0.0;
			// Find the largest contour
			//
			findContours(img_binsw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());

			// Schwarze Linie
			if(contours.size() > 0) {
				std::cout << "find line" << std::endl;
				drawContours(img_rgb, contours, -1, Scalar(255,0,50), 1);
				for (uint i = 0; i< contours.size(); i++)
				{
					double area = contourArea(contours[i]);
					if (area > maxArea)
					{
						maxArea = area;
						savedContour = i;
					}
				}
				// Create mask
				drawContours(result, contours, savedContour, Scalar(255), CV_FILLED, 8);

				// apply the mask:
				img_binsw &= result;
//				morphologyEx(img_binsw, img_binsw, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(8,8)));

				findContours(img_binsw(rois[0]), cont1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				findContours(img_binsw(rois[1]), cont2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				findContours(img_binsw(rois[2]), cont3, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				findContours(img_binsw(rois[3]), cont4, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				findContours(img_binsw(rois[4]), cont5, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				double grocont1 = 0;
				double grocont2 = 0;
				double grocont3 = 0;
				double grocont4 = 0;
				double grocont5 = 0;

				// Schwarze Linie
				if(cont1.size() > 0) {
					int index = 0;
					for(unsigned int i = 0; i < cont1.size(); i++) {
						Moments m = moments(cont1[i]);
						if(m.m00 > grocont1) 		//Größte Kontur finden und nutzen
						{
							index = i;
							grocont1 = m.m00;
							mitte1.x = m.m10 / m.m00;
						}
					}
					drawContours(img_rgb(rois[0]), cont1, index, Scalar(0,120,220), 1);
					circle(img_rgb(rois[0]), mitte1, 3, Scalar(255,140,10),2);
				}

				if(cont2.size() > 0) {
					int index = 0;
					for(unsigned int i = 0; i < cont2.size(); i++) {
						Moments m = moments(cont2[i]);
						if(m.m00 > grocont2)
						{
							index = i;
							grocont2 = m.m00;
							mitte2.x = m.m10 / m.m00;
						}
					}
					drawContours(img_rgb(rois[1]), cont2, index, Scalar(0,120,220), 1);
					circle(img_rgb(rois[1]), mitte2, 3, Scalar(255,140,10),2);
				}

				if(cont3.size() > 0) {
					int index = 0;
					for(unsigned int i = 0; i < cont3.size(); i++) {
						Moments m = moments(cont3[i]);
						if(m.m00 > grocont3)
						{
							index = i;
							grocont3 = m.m00;
							mitte3.x = m.m10 / m.m00;
						}
					}
					drawContours(img_rgb(rois[2]), cont3, index, Scalar(0,120,220), 1);
					circle(img_rgb(rois[2]), mitte3, 3, Scalar(255,140,10),2);
				}

				if(cont4.size() > 0) {
					int index = 0;
					for(unsigned int i = 0; i < cont4.size(); i++) {
						Moments m = moments(cont4[i]);
						if(m.m00 > grocont4)
						{
							index = i;
							grocont4 = m.m00;
							mitte4.x = m.m10 / m.m00;
						}
					}
					drawContours(img_rgb(rois[3]), cont4, index, Scalar(0,120,220),1);
					circle(img_rgb(rois[3]), mitte4, 3, Scalar(255,140,10),2);
				}

				if(cont5.size() > 0) {
					int index = 0;
					for(unsigned int i = 0; i < cont5.size(); i++) {
						Moments m = moments(cont5[i]);
						if(m.m00 > grocont5)
						{
							index = i;
							grocont4 = m.m00;
							mitte5.x = m.m10 / m.m00;
						}
					}
					drawContours(img_rgb(rois[4]), cont5, index, Scalar(0,120,220), 1);
					circle(img_rgb(rois[4]), mitte5, 3, Scalar(255,140,10),2);
				}

				/*
				 * GRÜN ERKENNUNG
				 *
				 */

//				Rect grroi = Rect(0,img_rgb.rows / 5 * 2,rois[0].width, rois[0].height + rois[1].height + rois[2].height);
				findContours(img_bin, contg, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				drawContours(img_rgb, contg, -1, Scalar(20, 255, 20), 1);

				std::cout << "Found " << contg.size() << " contour(s); ";
				for(unsigned int i = 0; i < contg.size(); i++) {
					Moments m = moments(contg[i]);
					Point mid;
					mid.x = m.m10 / m.m00;
					mid.y = m.m01 / m.m00;

					if(m.m00 < 500) {
						contg.erase(contg.begin()+i);
						i--;
						//				std::cerr << "GRPUNKT ZU KLEIN! (x=" << mid.x << ",y=" << mid.y << ") " << std::endl;
					} else if(mid.x > img_rgb.cols || mid.y > img_rgb.rows || mid.x <= 0 || mid.y <= 0) {
						contg.erase(contg.begin()+i);
						i--;
					}
				}

				std::cout << contg.size() << " correct one(s);" << std::endl;


				std::vector<Point> bpointsleft;
				std::vector<Point> bpointsright;
				std::vector<int> pointswidth;
				std::vector<int> pointsheight;

				if(contg.size() == 0) {
					grState = GRNOT;
					grDist = 0;
				}
				std::cout << "1" << std::endl;
				for(unsigned int i = 0 ; i < contg.size() && bildnr > 5; i++) {

					// PUNKTE FINDEN UND GUCKEN OB RELEVANT

					std::cout << "Grpunkt " << i << ":" << std::endl;

					Moments m = moments(contg[i]);
					mittegr.x = m.m10 / m.m00;
					mittegr.y = m.m01 / m.m00;

					std::cout << "2" << std::endl;

					int midtotop = 0;
					int black = 0;
					int wurzel = sqrt(m.m00);

					//Mittelpunkt bis obere Kante Schwarzer Punkt
					for(int i = 0; i < 2*wurzel && mittegr.y-i >= 0; ++i) {
						//std::cout << "vor pix check; x:" << mittegr.x << " y:" << mittegr.y << " y to check:" << (mittegr.y-i) << std::endl;
						int color = (int)img_bin.at<uchar>(mittegr.y-i,mittegr.x);
						//std::cout << "nach" << std::endl;
						if(color == 255) {
							midtotop++;
						} else {
							i = 2*wurzel;
						}
					}

					std::cout << "3" << std::endl;

					std::cout << "  Midtotop: " << midtotop << " wurzel: " << wurzel;

					//Checken wie viele Pixel über grPunkt schwarz sind
					for (int c = 0; c < 2*midtotop && (mittegr.y-c-midtotop) >= 0; ++c) {
						std::cout << "vor pix check; x:" << mittegr.x << " y:" << mittegr.y << "y to check:" << (mittegr.y-c-midtotop) << std::endl;
						int color = (int)img_binsw.at<uchar>((mittegr.y-c-midtotop),mittegr.x);
						std::cout << "nach" << std::endl;
						circle(img_rgb, Point(mittegr.x,mittegr.y-c-midtotop), 1, Scalar(10,255,10),1);

						if(color == 255) {
							++black;
						} else {
							if(black > 8 || c > 20) c = 2*midtotop;
						}
					}
					std::cout << ";  Blacktop:" << black;
					// Wenn schwarze Linie über grpunkt breit genug
					if(black > midtotop/2) {
						int bleft = 0;
						int bright = 0;
						int midtoright = 0;
						int midtoleft = 0;

						//Mitte bis linke Kante grPunkt
						for(int i = 0; i < 2*wurzel && mittegr.x-i >= 0; ++i) {
							int color = 0;
							color = (int)img_bin.at<uchar>(mittegr.y,mittegr.x-i);
							if(color == 255) {
								midtoleft++;
							} else {
								i = 2*wurzel;
							}
						}

						//Mitte bis rechte Kante grPunkt
						for(int i = 0; i < 2*wurzel && mittegr.x+i < 640; ++i) {
							if((int)img_bin.at<uchar>(mittegr.y,mittegr.x+i) == 255) {
								midtoright++;
							} else {
								i = 2*wurzel;
							}
						}

						//schwarze Linie links checken
						for (int l = 0; l < 2*wurzel && mittegr.x-l-midtoleft >= 0; l++) {
							int color = (int)img_binsw.at<uchar>(mittegr.y,(mittegr.x-l-midtoleft));

							if(color == 255) {
								++bleft;
							} else {
								if(bleft > 2 || l > 20) l = 2*wurzel;
							}
						}

						//schwarze Linie rechts checken
						for (int r = 0; r < 2*wurzel && mittegr.x+r+midtoright < 640; r++) {
							int color = (int)img_binsw.at<uchar>(mittegr.y,(mittegr.x+r+midtoright));

							if(color == 255) {
								++bright;
							} else {
								if(bright > 2 || r > 20) r = 2*wurzel;
							}
						}

						std::cout << ";  black Points right: " << bright << ", left: " <<  bleft << std::endl;
						circle(img_rgb, Point((mittegr.x-bleft-midtoleft),mittegr.y), 2, Scalar(255,10,10),2);
						circle(img_rgb, Point((mittegr.x+bright+midtoright),mittegr.y), 2, Scalar(255,10,10),2);
						circle(img_rgb, mittegr, 3, Scalar(10,255,10),2);
						//				grabbiegen[grpunktid]++;

						// auswerten auf welcher Seite der Linie der grPunkt liegt
						if(bleft > bright && bleft > midtoleft/2) {
							std::cout << "  Grünpunkt RECHTS" << std::endl;
							bpointsright.push_back(mittegr);
							grState = GRRIGHT;
							grDist = img_rgb.rows - mittegr.y;
						} else if(bright > bleft && bright > midtoright/2) {
							std::cout << "  Grünpunkt LINKS" << std::endl;
							bpointsleft.push_back(mittegr);
							pointswidth.push_back(midtoright*2);
							pointsheight.push_back(midtotop*2);
							grState = GRLEFT;
							grDist = img_rgb.rows - mittegr.y;
						} else {
							std::cout << "  Grünpunkt Seite nicht eindeutig!" << std::endl;
						}
					} else {
						grState = GRNOT;
						circle(img_rgb, mittegr, 3, Scalar(0,0,255),2);
					}
				}

				if(bpointsleft.size() > 0 && bpointsright.size() > 0) {
					double klein = -1;
					int indexr = -1;
					int indexl = -1;
					for(unsigned int l = 0; l < bpointsleft.size(); ++l) {
						for(unsigned int r = 0; r < bpointsright.size(); ++r) {
							int distance = sqrt(pow(pdif(bpointsleft[l].x-bpointsright[r].x),2) + pow(pdif(bpointsleft[l].y-bpointsright[r].y),2));
							if(distance < pointswidth[l] * 3 && pdif(bpointsleft[l].y - bpointsright[r].y) < pointsheight[l]*2) {
								if(klein == -1) {
									indexr = r;
									indexl = l;
									klein = distance;
								} else if(distance < klein) {
									indexr = r;
									indexl = l;
								}
							}
						}
						if(klein != -1) {
							line(img_rgb,bpointsleft[indexl],bpointsright[indexr],Scalar(255,0,0),2);
						}
					}
					if(indexl != -1 && indexr != -1) {
						grDist = img_rgb.rows - (bpointsleft[indexl].y + bpointsright[indexr].y) / 2;
						grState = GRBOTH;
						std::cout << "GR BOTH" << std::endl;
					} else if(indexl == -1) {
						int nearl = -1;
						int nearlidx = -1;
						int nearr = -1;
						int nearridx = -1;
						for(unsigned int l = 0; l < bpointsleft.size(); ++l) {
							if(nearlidx == -1) {
								nearlidx = l;
								nearl = bpointsleft[l].y;
							}
							if(bpointsleft[l].y > nearl) {
								nearlidx = l;
								nearl = bpointsleft[l].y;
							}
						}
						for(unsigned int r = 0; r < bpointsright.size(); ++r) {
							if(nearridx == -1) {
								nearridx = r;
								nearr = bpointsleft[r].y;
							}
							if(bpointsleft[r].y > nearr) {
								nearridx = r;
								nearr = bpointsleft[r].y;
							}
						}
						if(nearr > nearl) {
							grState = GRRIGHT;
							grDist = img_rgb.rows - bpointsright[nearridx].y;
							std::cout << "GR RIGHT" << std::endl;
						} else {
							grState = GRLEFT;
							grDist = img_rgb.rows - bpointsleft[nearlidx].y;
							std::cout << "GR LEFT" << std::endl;
						}
					}
				}
			}

			//Stelle alle gefundenen Konturen in grüner Farbe in "img_rgb" dar

			std::cout << "Line Point 1: " << mitte1.x << std::endl;
			std::cout << "Line Point 2: " << mitte2.x << std::endl;
			std::cout << "Line Point 3: " << mitte3.x << std::endl;
			std::cout << "Line Point 4: " << mitte4.x << std::endl;
			std::cout << "Line Point 5: " << mitte5.x << std::endl;
			std::cout << "Green State:" << grState << "; Pixel to Green:" << grDist << std::endl;

			char mid1tmp[3];
			char mid2tmp[3];
			char mid3tmp[3];
			char mid4tmp[3];
			char mid5tmp[3];
			char grpt[3];
			char togr[3];
			int null = 0;
			int mid1 = (int)mitte1.x;
			int mid2 = (int)mitte2.x;
			int mid3 = (int)mitte3.x;
			int mid4 = (int)mitte4.x;
			int mid5 = (int)mitte5.x;
			// check for 3 digits and write it in char array; else char arr = "0"
			if(rightLenght(mid5) && cont5.size() != 0) {
				std::sprintf(mid5tmp, "%d", mid5);
			} else {
				std::sprintf(mid5tmp, "%d", null);
			}
			if(rightLenght(mid4) && cont4.size() != 0) {
				std::sprintf(mid4tmp, "%d", mid4);
			} else {
				std::sprintf(mid4tmp, "%d", null);
			}
			if(rightLenght(mid3) && cont3.size() != 0) {
				std::sprintf(mid3tmp, "%d", mid3);
			} else {
				std::sprintf(mid3tmp, "%d", null);
			}
			if(rightLenght(mid2) && cont2.size() != 0) {
				std::sprintf(mid2tmp, "%d", mid2);
			} else {
				std::sprintf(mid2tmp, "%d", null);
			}
			if(rightLenght(mid1) && cont1.size() != 0) {
				std::sprintf(mid1tmp, "%d", mid1);
			} else {
				std::sprintf(mid1tmp, "%d", null);
			}
			if(rightLenght(grDist) && contg.size() != 0) {
				std::sprintf(togr, "%d", grDist);
			} else {
				std::sprintf(togr, "%d", null);
			}
			std::sprintf(grpt, "%d", grState);

			// Send data  ":data\ndata\ndata\n...!
			serialPuts(fd, ":");
			serialPuts(fd, mid1tmp);
			newLine(fd);
			serialPuts(fd, mid2tmp);
			newLine(fd);
			serialPuts(fd, mid3tmp);
			newLine(fd);
			serialPuts(fd, mid4tmp);
			newLine(fd);
			serialPuts(fd, mid5tmp);
			newLine(fd);
			serialPuts(fd, grpt);
			newLine(fd);
			serialPuts(fd, togr);
			newLine(fd);
			serialPuts(fd, "!");

			std::cout << "pin 5 High" << std::endl;

			std::cout << "data sended; "<< std::endl;

			//Übergebe Bilder an den Server
			srv.imshow("RGB", img_rgb);
			srv.imshow("Bin", img_bin);
			srv.imshow("SW", img_binunt);
			srv.imshow("BinSW", img_binsw);
			srv.imshow("HSV", img_hsv);
			srv.imshow("Hell", img_binhell);
			//Stoße die Übertragung an
			srv.update();

			digitalWrite(5, 1);
			senddata = false;
			int64_t t2 = cv::getTickCount();
			std::cout << "Bildauswertung und senden: " << (t2-t1) / cv::getTickFrequency() * 1000.0 << " ms" << std::endl;
		}
	}
	return 0;
}
