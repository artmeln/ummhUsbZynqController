#include <string>
#include <vector>
#include <math.h>
#include "xtime_l.h"

#include "xusbps_IniRxTx.h"

#include "uuhreserved.h"


// SD card
#include "xsdps.h"
#include "ff.h"
#include <fstream>

using namespace std;

#define CONTROLLER_DEBUG 1

/************ SD card definitions ************/
static FATFS FS_instance;			// File System instance
static FIL file1;					// File instance
int result;						// FRESULT variable
static const char *path = "0:/";	//  string pointer to the logical drive number
char filename[32] = "devices.txt";

/************ Simulated camera image definitions ************/
u32 const imgWidth = 400;
u32 const imgHeight = 300;
u32 roiX=0;
u32 roiY=0;
u32 roiW = imgWidth;
u32 roiH = imgHeight;
u32 binning = 1;
u32 exposure = 1;
u32 const imgSize = imgWidth*imgHeight;
u16 img[imgSize];		// 16bit version
u16 imgBinned[imgSize/4];
u16 imgRoi[imgSize];	// temporary ROI storage
u8 bytesPerPixel = 2;	// 16bit version

u8* buffImg;
static u32 sentImgBytes;
bool sendingImage = false;


/************ Simulated camera image definitions (B) ************/
u32 const imgWidth_B = 400;
u32 const imgHeight_B = 300;
u32 roiX_B=0;
u32 roiY_B=0;
u32 roiW_B = imgWidth_B;
u32 roiH_B = imgHeight_B;
u32 binning_B = 1;
u32 exposure_B = 1;
u32 const imgSize_B = imgWidth_B*imgHeight_B;
u16 img_B[imgSize_B];		// 16bit version
u16 imgBinned_B[imgSize_B/4];
u16 imgRoi_B[imgSize_B];	// temporary ROI storage
u8 bytesPerPixel_B = 2;	// 16bit version

u8* buffImg_B;
static u32 sentImgBytes_B;
bool sendingImage_B = false;

u32 CAMERA_TRANSFER_SIZE = 256*512;
u8 txEventCounterEp2=0;
u8 txEventCounterEp3=0;

/*** String constants for communicating with MM adapter ***/
string devicename;
string command;
vector<string> words;
vector<string> vals;
string val;
vector<string> emptyvs;

/*
 * Read a line delineated by quotation marks from a file.
 * Example: abcde"efgh"ijk will be read and returned as efgh
 * Returns 0 in case of success, error code otherwise
 */
int read_c_str_from_sd(FIL* file, char* c_str) {

	char* pointer = c_str;
	int result;
	char buf_ch[1];
	UINT nbr=0;	// number of bytes read

	// read to the first quotation mark
	result = f_read(file,(void*)buf_ch,1,&nbr);
	if (result) return result;
	while (nbr==1 && buf_ch[0]!=34) {
		result = f_read(file,(void*)buf_ch,1,&nbr);
	}
	if (nbr!=1) return 20;

	// read between quotation marks
	result = f_read(file,(void*)buf_ch,1,&nbr);
	while (nbr==1 && buf_ch[0]!=34) {
		*c_str = buf_ch[0];
		c_str++;
		result = f_read(file,(void*)buf_ch,1,&nbr);
	}
	if (nbr!=1) return 20;
	*c_str = '\0';
	c_str = pointer;
	return 0;

}

vector<string> SplitStringIntoWords(std::string line, char sep)
{
	// split the string into words
	vector<string> retwords;
	size_t pos1 = 0, pos2 = 0;
	pos1 = (size_t)-1;
	pos2 = line.find(sep, pos1 + 1);
	if (pos2==pos1) {
		retwords.push_back(line);
		return retwords;
	}
	while (pos2 != string::npos)
	{
		retwords.push_back(line.substr(pos1 + 1, pos2 - (pos1 + 1)));
		pos1 = pos2;
		pos2 = line.find(sep, pos1 + 1);
	}
	retwords.push_back(line.substr(pos1 + 1, line.length() - 1));
	return retwords;
}

void make_and_send_output_command(string devicename, string command, int error, vector<string> values) {
	devicename.push_back(uuhwords::sepIn);
	devicename.append(command);
	devicename.push_back(uuhwords::sepIn);
	devicename.append(to_string(error));
	for (size_t cc=0; cc<values.size(); cc++) {
		devicename.push_back(uuhwords::sepWithin);
		devicename.append(values.at(cc));
	}
	devicename.push_back(uuhwords::sepEnd);
	char str[64];
	strcpy(str,devicename.c_str());
	#ifdef CONTROLLER_DEBUG
		xil_printf("Sending string (Ep1):\r\n");
		xil_printf(str);
		xil_printf("\r\n");
	#endif
	SendToEp1((u8*)str, strlen(str));
}

// simulate camera image (16-bit version)
/*void simulate_image() {
	u16 val=0;
	for (u32 ii=posStart; ii<imgHeight; ii++) {
		for (u32 jj=0; jj<imgWidth; jj++) {
			img[ii*imgWidth+jj] = val;
		}
		val++;
	}
	for (u32 ii=0; ii<posStart; ii++) {
		for (u32 jj=0; jj<imgWidth; jj++) {
			img[ii*imgWidth+jj] = val;
		}
		val++;
	}
	posStart++;
	if (posStart==imgHeight) posStart=0;
}*/

// simulate camera image (16-bit version)
void simulate_image() {
	XTime curTime;
	XTime_GetTime(&curTime);

	int rho = imgHeight / 4;
	double phi = (2*M_PI*curTime)/0xFFFFFFFF;
	long xc = imgWidth/2 + lround(rho*cos(phi));
	long yc = imgHeight/2 + lround(rho*sin(phi));

	memset(img,0,imgSize*bytesPerPixel);
	double val=0;
	for (int rr=yc-25; rr<yc+25; rr++) {
		for (int cc=xc-25; cc<xc+25; cc++) {
			val = ((cc-xc)*(cc-xc)+(rr-yc)*(rr-yc))/(imgHeight/4.0)/(imgHeight/4.0)/2.0;
			val = exp(-val);
			img[rr*imgWidth+cc] = round(65535*val);
		}
	}

}

// apply ROI to the simulated image
void apply_roi() {

	if (roiX==0 && roiY==0 && roiW*binning==imgWidth && roiH*binning==imgHeight) return;
	u16* imgTemp;
	u16* imgRoiTemp;
	imgTemp = img + (roiY*imgWidth+roiX)*binning;
	imgRoiTemp = imgRoi;
	for (u32 rr=0; rr<roiH*binning; rr++) {
		memcpy(imgRoiTemp,imgTemp,roiW*binning*bytesPerPixel);
		imgTemp = imgTemp + imgWidth;
		imgRoiTemp = imgRoiTemp + roiW*binning;
	}
	memcpy(img,imgRoi,roiW*binning*roiH*binning*bytesPerPixel);
}

// apply binning to the ROI image
void apply_binning() {
	if (binning==1) return;
	float v;
	u32 ii=0;
	u32 ccS, rrS;
	rrS = 0;
	while (rrS<roiH*binning) {
		ccS = 0;
		while (ccS<roiW*binning) {
			v = 0;
			for (u32 rr=rrS; rr<(rrS+binning); rr++) {
				for (u32 cc=ccS; cc<(ccS+binning); cc++) {
					v += img[rr*roiW*binning+cc];
				}
			}
			if (v>65535) v=65535;
			imgBinned[ii] = lround(v);
			ii++;
			ccS+=binning;
		}
		rrS+=binning;
	}
	memcpy(img,imgBinned,roiW*roiH*bytesPerPixel);
}

// simulate camera image (16-bit version) (B)
void simulate_image_B() {
	XTime curTime;
	XTime_GetTime(&curTime);

	int rho = imgHeight_B / 4;
	double phi =  - (2*M_PI*curTime)/0xFFFFFFFF;
	long xc = imgWidth_B/2 + lround(rho*cos(phi));
	long yc = imgHeight_B/2 + lround(rho*sin(phi));

	memset(img_B,0,imgSize_B*bytesPerPixel_B);
	double val=0;
	for (int rr=yc-25; rr<yc+25; rr++) {
		for (int cc=xc-25; cc<xc+25; cc++) {
			val = ((cc-xc)*(cc-xc)+(rr-yc)*(rr-yc))/(imgHeight_B/4.0)/(imgHeight_B/4.0)/2.0;
			val = exp(-val);
			img_B[rr*imgWidth_B+cc] = round(65535*val);
		}
	}

}

// apply ROI to the simulated image (B)
void apply_roi_B() {

	if (roiX_B==0 && roiY_B==0 && roiW_B*binning_B==imgWidth_B && roiH_B*binning_B==imgHeight_B) return;
	u16* imgTemp;
	u16* imgRoiTemp;
	imgTemp = img_B + (roiY_B*imgWidth_B+roiX_B)*binning_B;
	imgRoiTemp = imgRoi_B;
	for (u32 rr=0; rr<roiH_B*binning_B; rr++) {
		memcpy(imgRoiTemp,imgTemp,roiW_B*binning_B*bytesPerPixel_B);
		imgTemp = imgTemp + imgWidth_B;
		imgRoiTemp = imgRoiTemp + roiW_B*binning_B;
	}
	memcpy(img_B,imgRoi_B,roiW_B*binning_B*roiH_B*binning_B*bytesPerPixel_B);
}

// apply binning to the ROI image (B)
void apply_binning_B() {
	if (binning_B==1) return;
	float v;
	u32 ii=0;
	u32 ccS, rrS;
	rrS = 0;
	while (rrS<roiH_B*binning_B) {
		ccS = 0;
		while (ccS<roiW_B*binning_B) {
			v = 0;
			for (u32 rr=rrS; rr<(rrS+binning_B); rr++) {
				for (u32 cc=ccS; cc<(ccS+binning_B); cc++) {
					v += img_B[rr*roiW_B*binning_B+cc];
				}
			}
			if (v>65535) v=65535;
			imgBinned_B[ii] = lround(v);
			ii++;
			ccS+=binning_B;
		}
		rrS+=binning_B;
	}
	memcpy(img_B,imgBinned_B,roiW_B*roiH_B*bytesPerPixel_B);
}


int main(void)
{
	int Status;

	// initialize usb2
	Status = SetupUsbDevice(&txEventCounterEp2, &txEventCounterEp3);
	if (Status != XST_SUCCESS) {
		#ifdef CONTROLLER_DEBUG
			xil_printf("Failed to setup USB2.\r\n");
		#endif
		return XST_FAILURE;
	}

	// mount the sd card
	Status = f_mount(&FS_instance,path, 1);
	if (Status != XST_SUCCESS) {
		#ifdef CONTROLLER_DEBUG
			xil_printf("Failed to mount the SD card.\r\n");
		#endif
		return XST_FAILURE;
	}

	u8* rxBuffEp1;
	u8 lenRxBuffEp1;
	string str;
	char c_str[512];

	// initial state of shutters A and B
	bool sh_open_A = false;

	while (1) {
		ReadFromEp1(&rxBuffEp1, &lenRxBuffEp1);
		if (lenRxBuffEp1!=0) {
			// there is a new command on Ep1
			rxBuffEp1[lenRxBuffEp1] = '\0';
			#ifdef CONTROLLER_DEBUG
				xil_printf("Received string (Ep1):\r\n");
				xil_printf((char*)rxBuffEp1);
				xil_printf("\r\n");
			#endif
			// process the incoming command
	    	str = string((char*)rxBuffEp1);
	    	ResetRxBufferEp1();
	    	str = str.substr(0,str.find_first_of(uuhwords::sepEnd,0));

	    	if (str.size()>0) {
	    		// handle special one-word commands first
	        	if (strcmp(str.c_str(),uuhwords::device_list_start)==0) {
	        		// start passing device descriptions
	        		Status = f_open(&file1, filename, FA_READ);
	        		Status = read_c_str_from_sd(&file1,c_str);
					#ifdef CONTROLLER_DEBUG
	        			xil_printf("Sending string (Ep1):\r\n");
	        			xil_printf(c_str);
	        			xil_printf("\r\n");
					#endif
	        		SendToEp1((u8*)c_str, strlen(c_str));
	        		continue;
	        	} else if (strcmp(str.c_str(),uuhwords::device_list_continue)==0) {
	        		result = read_c_str_from_sd(&file1,c_str);
					#ifdef CONTROLLER_DEBUG
	        			xil_printf("Sending string (Ep1):\r\n");
	        			xil_printf(c_str);
	        			xil_printf("\r\n");
					#endif
	        		SendToEp1((u8*)c_str, strlen(c_str));
	        		if (strcmp(c_str,uuhwords::device_list_end)==0) f_close(&file1);
	        		continue;
	        	}
	        	// handle regular commands
	        	words = SplitStringIntoWords(str,uuhwords::sepOut);
	        	if (words.size()==3) {
	        		devicename = words[0];
	        		command = words[1];
	        		vals = SplitStringIntoWords(words[2],uuhwords::sepWithin);
	        		if (strcmp(devicename.c_str(),"Shutter-A")==0) {
	    				if (strcmp(command.c_str(),"SO")==0) {
	    					if (strcmp(vals[0].c_str(),"0")==0) {
	    						sh_open_A = false;
	    					} else {
	    						sh_open_A = true;
	    					}
	    					vals.clear();
	    					vals.push_back(to_string(sh_open_A));
							make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"SF")==0) {
	        				make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"SI")==0) {
	        				make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"SS")==0) {
	        				make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"DL1")==0) {
	        				sleep(2);
	        				make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"DL2")==0) {
	        				vector<string> inivals = vals;
	    					vals.clear();
	    					vals.push_back(to_string(2500));
	        				make_and_send_output_command(devicename,uuhwords::timeout,1,vals);
	        				sleep(2);
	        				make_and_send_output_command(devicename,command,0,inivals);
	    					vals.clear();
	    					vals.push_back(to_string(1000));
	        				make_and_send_output_command(devicename,uuhwords::timeout,0,vals);
	        			} else if (strcmp(command.c_str(),"RB0.1")==0) {
	        				float v = atof(vals[0].c_str());
	        				v = v - 0.1;
	        				vals.clear();
	        				vals.push_back(to_string(v));
	        				make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"UBCnot")==0) {
	        				make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"OOLF")==0) {
	        				float v = 10.0;
	        				vals.clear();
	        				vals.push_back(to_string(v));
	        				make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"OOLI")==0) {
	        				int v = 10.0;
	        				vals.clear();
	        				vals.push_back(to_string(v));
	        				make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"OOLS")==0) {
	        				string v("H");
	        				vals.clear();
	        				vals.push_back(v);
	        				make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"SBNR")==0) {
	        				make_and_send_output_command(devicename,command,1,vals);
	        			} else {
	        				make_and_send_output_command(devicename,command,uuherrors::ctr_device_command_not_recognized,emptyvs);
	        			}
	        		}
	        		else if (strcmp(devicename.c_str(),"State-A")==0) {
	    				if (strcmp(command.c_str(),"SST")==0) {
							make_and_send_output_command(devicename,command,0,vals);
	        			} else {
	        				make_and_send_output_command(devicename,command,uuherrors::ctr_device_command_not_recognized,emptyvs);
	        			}
	        		}
	        		else if (strcmp(devicename.c_str(),"Stage-A")==0) {
	    				if (strcmp(command.c_str(),"SP")==0) {
							make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"HM")==0) {
	    					vals.clear();
	    					vals.push_back(to_string(0.0));
							make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"STOP")==0) {
	    					vals.clear();
	    					vals.push_back(to_string(666.666));
							make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"MD")==0) {
							make_and_send_output_command(devicename,command,0,vals);
	        			} else {
	        				make_and_send_output_command(devicename,command,uuherrors::ctr_device_command_not_recognized,emptyvs);
	        			}
	        		}
	        		else if (strcmp(devicename.c_str(),"XYStage-A")==0) {
	    				if (strcmp(command.c_str(),"SP")==0) {
							make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"HM")==0) {
	    					vals.clear();
	    					vals.push_back(to_string(0.0));
	    					vals.push_back(to_string(0.0));
							make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"STOP")==0) {
	    					vals.clear();
	    					vals.push_back(to_string(666.666));
	    					vals.push_back(to_string(777.777));
							make_and_send_output_command(devicename,command,0,vals);
	        			} else if (strcmp(command.c_str(),"MD")==0) {
							make_and_send_output_command(devicename,command,0,vals);
	        			} else {
	        				make_and_send_output_command(devicename,command,uuherrors::ctr_device_command_not_recognized,emptyvs);
	        			}
	        		}
        			else if (strcmp(devicename.c_str(),"Camera-A")==0) {
        				if (strcmp(command.c_str(),"SI")==0) {
        					simulate_image();
        					apply_roi();
        					apply_binning();
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"GIB")==0) {
        					buffImg = (u8*)img;
        					sentImgBytes = 0;
        					sendingImage = true;
        					// no response is sent here but
        					// image transfer will begin after
        					// input command has been processed
        					// (see usage of sendingImage flag)
        				} else if (strcmp(command.c_str(),"DN")==0) {
        					sleep(2);
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"EX")==0) {
        					u32 newExposure = atoi(vals.at(0).c_str());
        					if (newExposure<1 || newExposure>10000) newExposure = exposure;
        					exposure = newExposure;
    						vals.clear();
    						itoa(exposure,c_str,10);
    						vals.push_back(string(c_str));
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"SB")==0) {
        					if (sendingImage) {
        						make_and_send_output_command(devicename,command,uuherrors::ctr_busy,emptyvs);
        					}
        					u32 newBinning = atoi(vals.at(0).c_str());
        					if (newBinning<=0 || newBinning>4) newBinning = binning;
		        			roiX = (roiX*binning)/newBinning;
		        			roiY = (roiY*binning)/newBinning;
        					roiW = (roiW*binning)/newBinning;
        					roiH = (roiH*binning)/newBinning;
        					binning = newBinning;
    						vals.clear();
    						itoa(binning,c_str,10);
    						vals.push_back(string(c_str));
        					make_and_send_output_command(devicename,command,0,vals);
        					// update ROI
    						vals.clear();
    						itoa(roiX,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiY,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiW,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiH,c_str,10);
    						vals.push_back(string(c_str));
        					make_and_send_output_command(devicename,"SR",0,vals);
        				} else if (strcmp(command.c_str(),"SR")==0) {
        					if (sendingImage) {
        						make_and_send_output_command(devicename,command,uuherrors::ctr_busy,emptyvs);
        					}
        					roiX = atoi(vals.at(0).c_str());
        					roiY = atoi(vals.at(1).c_str());
        					roiW = atoi(vals.at(2).c_str());
        					roiH = atoi(vals.at(3).c_str());
        					if (roiW<=0 || roiW>imgWidth/binning ||
        						roiH<=0 || roiH>imgHeight/binning ||
								roiX<0 || roiX>imgWidth/binning || (roiX+roiW)>imgWidth/binning ||
								roiY<0 || roiY>imgHeight/binning || (roiY+roiH)>imgHeight/binning) {
        						roiX=0;
        						roiY=0;
        						roiW = imgWidth/binning;
        						roiH = imgHeight/binning;
        					}
    						vals.clear();
    						itoa(roiX,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiY,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiW,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiH,c_str,10);
    						vals.push_back(string(c_str));
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"CR")==0) {
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"TT")==0) {
        					make_and_send_output_command(devicename,command,0,vals);
        				} else {
        					make_and_send_output_command(devicename,command,uuherrors::ctr_device_command_not_recognized,emptyvs);
        				}
        			}
        			else if (strcmp(devicename.c_str(),"Camera-B")==0) {
        				if (strcmp(command.c_str(),"SI")==0) {
        					simulate_image_B();
        					apply_roi_B();
        					apply_binning_B();
        					usleep(exposure_B*1000);
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"GIB")==0) {
        					buffImg_B = (u8*)img_B;
        					sentImgBytes_B = 0;
        					sendingImage_B = true;
        					// no response is sent here but
        					// image transfer will begin after
        					// input command has been processed
        					// (see usage of sendingImage flag)
        				} else if (strcmp(command.c_str(),"DN")==0) {
        					sleep(2);
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"EX")==0) {
        					u32 newExposure = atoi(vals.at(0).c_str());
        					if (newExposure<1 || newExposure>10000) newExposure = exposure_B;
        					exposure_B = newExposure;
    						vals.clear();
    						itoa(exposure_B,c_str,10);
    						vals.push_back(string(c_str));
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"SB")==0) {
        					if (sendingImage_B) {
        						make_and_send_output_command(devicename,command,uuherrors::ctr_busy,emptyvs);
        					}
        					u32 newBinning = atoi(vals.at(0).c_str());
        					if (newBinning<=0 || newBinning>4) newBinning = binning_B;
		        			roiX_B = (roiX_B*binning_B)/newBinning;
		        			roiY_B = (roiY_B*binning_B)/newBinning;
        					roiW_B = (roiW_B*binning_B)/newBinning;
        					roiH_B = (roiH_B*binning_B)/newBinning;
        					binning_B = newBinning;
    						vals.clear();
    						itoa(binning_B,c_str,10);
    						vals.push_back(string(c_str));
        					make_and_send_output_command(devicename,command,0,vals);
        					// update ROI
    						vals.clear();
    						itoa(roiX_B,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiY_B,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiW_B,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiH_B,c_str,10);
    						vals.push_back(string(c_str));
        					make_and_send_output_command(devicename,"SR",0,vals);
        				} else if (strcmp(command.c_str(),"SR")==0) {
        					if (sendingImage) {
        						make_and_send_output_command(devicename,command,uuherrors::ctr_busy,emptyvs);
        					}
        					roiX_B = atoi(vals.at(0).c_str());
        					roiY_B = atoi(vals.at(1).c_str());
        					roiW_B = atoi(vals.at(2).c_str());
        					roiH_B = atoi(vals.at(3).c_str());
        					if (roiW_B<=0 || roiW_B>imgWidth_B/binning_B ||
        						roiH_B<=0 || roiH_B>imgHeight_B/binning_B ||
								roiX_B<0 || roiX_B>imgWidth_B/binning_B || (roiX_B+roiW_B)>imgWidth_B/binning_B ||
								roiY_B<0 || roiY_B>imgHeight_B/binning_B || (roiY_B+roiH_B)>imgHeight_B/binning_B) {
        						roiX_B=0;
        						roiY_B=0;
        						roiW_B = imgWidth_B/binning_B;
        						roiH_B = imgHeight_B/binning_B;
        					}
    						vals.clear();
    						itoa(roiX_B,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiY_B,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiW_B,c_str,10);
    						vals.push_back(string(c_str));
    						itoa(roiH_B,c_str,10);
    						vals.push_back(string(c_str));
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"CR")==0) {
        					make_and_send_output_command(devicename,command,0,vals);
        				} else if (strcmp(command.c_str(),"TT")==0) {
        					make_and_send_output_command(devicename,command,0,vals);
        				} else {
        					make_and_send_output_command(devicename,command,uuherrors::ctr_device_command_not_recognized,emptyvs);
        				}
        			}
	        		else {
	        			make_and_send_output_command(devicename,command,uuherrors::ctr_device_not_recognized,emptyvs);
	        		}
	        	} else {
	        		devicename = string("Controller");
	        		command = string("ERR");
	        		make_and_send_output_command(devicename,command,uuherrors::ctr_string_not_recognized,emptyvs);
	        	}
	        }
	    }
		// continue sending camera image
    	if (sendingImage) {
    		int transferSize;
    		if (sentImgBytes<roiW*roiH*bytesPerPixel) {
    			if (roiW*roiH*bytesPerPixel-sentImgBytes>CAMERA_TRANSFER_SIZE) {
    				transferSize = CAMERA_TRANSFER_SIZE;
    			 } else {
    				 transferSize = roiW*roiH*bytesPerPixel-sentImgBytes;
    			 }
    			 txEventCounterEp2 = 0;
    			 SendToEp2(buffImg, transferSize);
    			 while (txEventCounterEp2<(transferSize+16*1024-1)/(16*1024)) {usleep(10);} // will be reset by interrupt
    			 sentImgBytes += transferSize;
    			 buffImg += transferSize;
    		 } else {
    			 // finished sending the image
    			 sendingImage = false;
    		 }
    	 }
    	 if (sendingImage_B) {
    		 int transferSize;
    		 if (sentImgBytes_B<roiW_B*roiH_B*bytesPerPixel_B) {
    			 if (roiW_B*roiH_B*bytesPerPixel_B-sentImgBytes_B>CAMERA_TRANSFER_SIZE) {
    				 transferSize = CAMERA_TRANSFER_SIZE;
    			 } else {
    				 transferSize = roiW_B*roiH_B*bytesPerPixel_B-sentImgBytes_B;
    			 }
    			 txEventCounterEp3 = 0;
    			 SendToEp3(buffImg_B, transferSize);
    			 while (txEventCounterEp3<(transferSize+16*1024-1)/(16*1024)) {usleep(10);} // will be reset by interrupt
    			 sentImgBytes_B += transferSize;
    			 buffImg_B += transferSize;
    		 } else {
    			 // finished sending the image
    			 sendingImage_B = false;
    		 }
    	 }
	}

	return XST_SUCCESS;
}

