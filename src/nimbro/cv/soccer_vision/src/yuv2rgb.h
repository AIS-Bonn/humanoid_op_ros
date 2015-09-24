// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#ifndef YUV2RGB_H
#define YUV2RGB_H

//--------------------------------------------------------------------
// saturate input into [0, 255]
inline unsigned char saturateValue(float f)
{
  return (unsigned char)( f >= 255 ? 255 : (f < 0 ? 0 : f));
}
//--------------------------------------------------------------------

inline void yuv2rgb(unsigned char yuvimg[], unsigned char rgbbuffer[], int W, int H){


	for(int i=0; i<(W*H*3); i+=3){

		rgbbuffer[i+2] = saturateValue( yuvimg[i]+1.402f  *(yuvimg[i+2]-128) );
		rgbbuffer[i+1] = saturateValue( yuvimg[i]-0.34414f*(yuvimg[i+1]-128)-0.71414f*(yuvimg[i+2]-128) );
		rgbbuffer[i] =   saturateValue( yuvimg[i]+1.772f  *(yuvimg[i+1]-128) );
	}

}


#endif

