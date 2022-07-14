#include <Wire.h>
#include "KX126.h"
#include "BM1422AGMV.h"
#include "BM1383AGLV.h"
#include <SDHCI.h>
#include <stdio.h> /* for sprintf */
#include <avr/pgmspace.h>

#include <Camera.h>

#define BAUDRATE (115200)
#define TOTAL_PICTURE_COUNT (10)

SDClass theSD;
int take_picture_count = 0;
BM1383AGLV bm1383aglv;
BM1422AGMV bm1422agmv(BM1422AGMV_DEVICE_ADDRESS_0F);
KX126 KX126(KX126_DEVICE_ADDRESS_1F);
void printError(enum CamErr err)
{
  Serial.print("Error: ");
  switch (err)
  {
  case CAM_ERR_NO_DEVICE:
    Serial.println("No Device");
    break;
  case CAM_ERR_ILLEGAL_DEVERR:
    Serial.println("Illegal device error");
    break;
  case CAM_ERR_ALREADY_INITIALIZED:
    Serial.println("Already initialized");
    break;
  case CAM_ERR_NOT_INITIALIZED:
    Serial.println("Not initialized");
    break;
  case CAM_ERR_NOT_STILL_INITIALIZED:
    Serial.println("Still picture not initialized");
    break;
  case CAM_ERR_CANT_CREATE_THREAD:
    Serial.println("Failed to create thread");
    break;
  case CAM_ERR_INVALID_PARAM:
    Serial.println("Invalid parameter");
    break;
  case CAM_ERR_NO_MEMORY:
    Serial.println("No memory");
    break;
  case CAM_ERR_USR_INUSED:
    Serial.println("Buffer already in use");
    break;
  case CAM_ERR_NOT_PERMITTED:
    Serial.println("Operation not permitted");
    break;
  default:
    break;
  }
}

/**
 * Callback from Camera library when video frame is captured.
 */

void CamCB(CamImage img)
{

  /* Check the img instance is available or not. */

  if (img.isAvailable())
  {

    /* If you want RGB565 data, convert image data format to RGB565 */

    img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);

    /* You can use image data directly by using getImgSize() and getImgBuff().
     * for displaying image to a display, etc. */

    // Serial.print("Image data size = ");
    // Serial.print(img.getImgSize(), DEC);
    // Serial.print(" , ");

    // Serial.print("buff addr = ");
    // Serial.print((unsigned long)img.getImgBuff(), HEX);
    // Serial.println("");
  }
  else
  {
    Serial.println("Failed to get video stream image");
  }
}
void setup()
{
  byte rc;
  byte rc1;
  byte rc2;

  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin();

  rc = KX126.init();
  if (rc != 0)
  {
    Serial.println("KX126 initialization failed");
    Serial.flush();
  }
  rc1 = bm1422agmv.init();
  if (rc != 0)
  {
    Serial.println(F("BM1422AGMV initialization failed"));
    Serial.flush();
  }
  rc2 = bm1383aglv.init();
  if (rc2 != 0)
  {
    Serial.println("BM1383AGLV initialization failed");
    Serial.flush();
  }
  CamErr err;

  /* Initialize SD */
  while (!theSD.begin())
  {
    /* wait until SD card is mounted. */
    Serial.println("Insert SD card.");
  }

  /* begin() without parameters means that
   * number of buffers = 1, 30FPS, QVGA, YUV 4:2:2 format */

  Serial.println("Prepare camera");
  err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

  /* Start video stream.
   * If received video stream data from camera device,
   *  camera library call CamCB.
   */

  Serial.println("Start streaming");
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

  /* Auto white balance configuration */

  Serial.println("Set Auto white balance parameter");
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

  /* Set parameters about still picture.
   * In the following case, QUADVGA and JPEG.
   */

  Serial.println("Set still picture format");
  err = theCamera.setStillPictureImageFormat(
      CAM_IMGSIZE_QUADVGA_H,
      CAM_IMGSIZE_QUADVGA_V,
      CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }
}
int state = 0;
float gmvX = 0;
float gmvY = 0;
float gmvZ = 0;
void loop()
{
  byte rc;
  float acc[3];

  //もし0なら写真を撮ってもいいことにする 1の時は写真を撮れない

  rc = KX126.get_val(acc);
  if (rc == 0)
  {
    //    Serial.write("KX126 (X) = ");
    //    Serial.print(acc[0]);
    //    Serial.println(" [g]");
    //    Serial.write("KX126 (Y) = ");
    //    Serial.print(acc[1]);
    //    Serial.println(" [g]");
    //    Serial.write("KX126 (Z) = ");
    //    Serial.print(acc[2]);
    //    Serial.println(" [g]");
    //    Serial.println();
    byte rc1;
    float mag[3];
    rc1 = bm1422agmv.get_val(mag);
    float gmvspd = 0;


    if (rc1 == 0)
    {
      gmvspd = (mag[0]-gmvX)*(mag[0]-gmvX) + (mag[1]-gmvY)*(mag[1]-gmvY) + (mag[2]-gmvZ)*(mag[2]-gmvZ);
      // Serial.print("BM1422AGMV XDATA=");
      // Serial.print(mag[0], 3);
      // Serial.println("[uT]");
      // Serial.print("BM1422AGMV YDATA=");
      // Serial.print(mag[1], 3);
      // Serial.println("[uT]");
      // Serial.print("BM1422AGMV ZDATA=");
      // Serial.print(mag[2], 3);
      // Serial.println("[uT]");
      Serial.println(gmvspd);
      gmvX = mag[0];
      gmvY = mag[1];
      gmvZ = mag[2];
    }
    float spd = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];
    if (spd < 1.1 && gmvspd < 5.0)
    {
      // Serial.println(state);
      Serial.println("静止してます");
      if (state == 0)
      {
        Serial.println("シャッター切りました．");
        state = 1;
        if (take_picture_count < TOTAL_PICTURE_COUNT)
        {

          /* Take still picture.
           * Unlike video stream(startStreaming) , this API wait to receive image data
           *  from camera device.
           */

          Serial.println("call takePicture()");
          CamImage img = theCamera.takePicture();

          /* Check availability of the img instance. */
          /* If any errors occur, the img is not available. */

          if (img.isAvailable())
          {
            Serial.println(img.getWidth());
            Serial.println(img.getHeight());
            Serial.println(img.getPixFormat());
            Serial.println(img.getImgSize());
            
            long l = pgm_read_dword(img.getImgBuff());
            Serial.println(l);
            /* Create file name */

            char filename[16] = {0};
            sprintf(filename, "PICT%03d.JPG", take_picture_count);

            // Serial.println(img);
            Serial.print("Save taken picture as ");
            Serial.print(filename);
            Serial.println("");

            /* Remove the old file with the same file name as new created file,
             * and create new file.
             */

            theSD.remove(filename);
            File myFile = theSD.open(filename, FILE_WRITE);
            myFile.write(img.getImgBuff(), img.getImgSize());
            myFile.close();
          }
          else
          {
            /* The size of a picture may exceed the allocated memory size.
             * Then, allocate the larger memory size and/or decrease the size of a picture.
             * [How to allocate the larger memory]
             * - Decrease jpgbufsize_divisor specified by setStillPictureImageFormat()
             * - Increase the Memory size from Arduino IDE tools Menu
             * [How to decrease the size of a picture]
             * - Decrease the JPEG quality by setJPEGQuality()
             */

            Serial.println("Failed to take picture");
          }
          if (take_picture_count == 9)
          {
            take_picture_count = -1;
          }
        }
        else
        {
          take_picture_count = -1;
        }

        take_picture_count++;
      }
    }

    else
    {
      Serial.println("静止して下さい");
      state = 0;
    }
    //    Serial.println(spd);
  }

  delay(500);
}
