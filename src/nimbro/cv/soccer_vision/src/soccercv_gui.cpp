// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#include <stdio.h>
#include <stdlib.h>
#include "gtk/gtk.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <image_transport/image_transport.h>

//Asuming that the original image is 800x600 (WxH)
#define ORG_IMAGE_WIDTH 800
#define ORG_IMAGE_HEIGTH 600
#define SUB_SAMPLING_PARAMETER 4
#define SUB_SAMPLING_WIDTH ORG_IMAGE_WIDTH/SUB_SAMPLING_PARAMETER
#define SUB_SAMPLING_HEIGTH ORG_IMAGE_HEIGTH/SUB_SAMPLING_PARAMETER

#define TOTAL_SUB_IMAGES 8

guchar imgorgbuffer[ORG_IMAGE_WIDTH*ORG_IMAGE_HEIGTH*3];
guchar imgorgsegmented[ORG_IMAGE_WIDTH*ORG_IMAGE_HEIGTH*3];
guchar subimages1[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGTH*3];

bool MAINLOOP_CONTROL = true;



struct orgsizeimage {
	GtkWidget *label;
    GtkWidget * image_widget;
    GdkPixbuf * pixbuf;
	guchar img[ORG_IMAGE_WIDTH*ORG_IMAGE_HEIGTH*3];
};


struct subimage {
	GtkWidget *label;
    GtkWidget * smallimage_widget;
    GdkPixbuf *pixbufsmall;
	guchar img[SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGTH*3];
};



subimage allSubimages[TOTAL_SUB_IMAGES];
orgsizeimage allOrgimages[2];


static gboolean delete_event( GtkWidget *widget,
                              GdkEvent  *event,
                              gpointer   data )
{
	MAINLOOP_CONTROL = false;
    //gtk_main_quit ();
    return FALSE;
}


void callbackGetFrame0(const sensor_msgs::Image::ConstPtr &msg){

	   for (unsigned int i = 0; i<= msg->width*msg->height*3; i+=3){
		   imgorgbuffer[i] = (guchar)msg->data[i+2];
		   imgorgbuffer[i+1] = (guchar)msg->data[i+1];
		   imgorgbuffer[i+2] = (guchar)msg->data[i];
	   }//END for
}

void callbackGetFrame00(const sensor_msgs::Image::ConstPtr &msg){

	   unsigned int j=0;
	   for (unsigned int i = 0; i<= msg->width*msg->height; i+=1){
		   j = 3*i;
		   imgorgsegmented[j] = (guchar)msg->data[i];
		   imgorgsegmented[j+1] = (guchar)msg->data[i];
		   imgorgsegmented[j+2] = (guchar)msg->data[i];
	   }//END for
}


void callbackGetFrame1(const sensor_msgs::Image::ConstPtr &msg){

	   unsigned int j=0;
	   unsigned int imgIndex = 0;
	   for (unsigned int i = 0; i<= msg->width*msg->height; i+=1){
		    j = 3*i;
		    allSubimages[imgIndex].img[j]   = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+1] = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+2] = (guchar)msg->data[i];
	   }//END for
}

void callbackGetFrame2(const sensor_msgs::Image::ConstPtr &msg){

	   unsigned int j=0;
	   unsigned int imgIndex = 1;
	   for (unsigned int i = 0; i<= msg->width*msg->height; i+=1){
		    j = 3*i;
		    allSubimages[imgIndex].img[j]   = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+1] = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+2] = (guchar)msg->data[i];
	   }//END for

}

void callbackGetFrame3(const sensor_msgs::Image::ConstPtr &msg){

	   unsigned int j=0;
	   unsigned int imgIndex = 2;
	   for (unsigned int i = 0; i<= msg->width*msg->height; i+=1){
		    j = 3*i;
		    allSubimages[imgIndex].img[j]   = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+1] = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+2] = (guchar)msg->data[i];
	   }//END for
}

void callbackGetFrame4(const sensor_msgs::Image::ConstPtr &msg){

	   unsigned int j=0;
	   unsigned int imgIndex = 3;
	   for (unsigned int i = 0; i<= msg->width*msg->height; i+=1){
		    j = 3*i;
		    allSubimages[imgIndex].img[j]   = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+1] = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+2] = (guchar)msg->data[i];
	   }//END forimgorgsegmented
}

void callbackGetFrame5(const sensor_msgs::Image::ConstPtr &msg){

	   unsigned int j=0;
	   unsigned int imgIndex = 4;
	   for (unsigned int i = 0; i<= msg->width*msg->height; i+=1){
		    j = 3*i;
		    allSubimages[imgIndex].img[j]   = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+1] = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+2] = (guchar)msg->data[i];
	   }//END for
}

void callbackGetFrame6(const sensor_msgs::Image::ConstPtr &msg){

	   unsigned int j=0;
	   unsigned int imgIndex = 5;
	   for (unsigned int i = 0; i<= msg->width*msg->height; i+=1){
		    j = 3*i;
		    allSubimages[imgIndex].img[j]   = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+1] = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+2] = (guchar)msg->data[i];
	   }//END for
}

void callbackGetFrame7(const sensor_msgs::Image::ConstPtr &msg){

	   unsigned int j=0;
	   unsigned int imgIndex = 6;
	   for (unsigned int i = 0; i<= msg->width*msg->height; i+=1){
		    j = 3*i;
		    allSubimages[imgIndex].img[j]   = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+1] = (guchar)msg->data[i];
		    allSubimages[imgIndex].img[j+2] = (guchar)msg->data[i];
	   }//END for
}


int main( int   argc, char *argv[])
{

	ros::init(argc, argv, "SoccerVisionGui");
    ros::NodeHandle nopVisonNode;

    image_transport::ImageTransport it(nopVisonNode);

    image_transport::Subscriber image_sub0_  = it.subscribe("image/rgb", 1, callbackGetFrame0);
	image_transport::Subscriber image_sub00_ = it.subscribe("image/segmented", 1, callbackGetFrame00);
    image_transport::Subscriber image_sub1_  = it.subscribe("image/smallimg1", 1, callbackGetFrame1);
    image_transport::Subscriber image_sub2_  = it.subscribe("image/smallimg2", 1, callbackGetFrame2);
    image_transport::Subscriber image_sub3_  = it.subscribe("image/smallimg3", 1, callbackGetFrame3);
    image_transport::Subscriber image_sub4_  = it.subscribe("image/smallimg4", 1, callbackGetFrame4);
    image_transport::Subscriber image_sub5_  = it.subscribe("image/smallimg5", 1, callbackGetFrame5);
    image_transport::Subscriber image_sub6_  = it.subscribe("image/smallimg6", 1, callbackGetFrame6);
	image_transport::Subscriber image_sub7_  = it.subscribe("image/smallimg7", 1, callbackGetFrame7);




    GtkWidget *window;
    GtkWidget *box1;
    GtkWidget *separator;
    GtkWidget *separator2;
    GtkWidget *label;
	GtkWidget *labelSegmented;
    GtkWidget *orgimage_widget;
    GdkPixbuf *orgpixbuf;
    GtkWidget *orgimage_widgetSegmented;
    GdkPixbuf *orgpixbufSegmented;
	
    char buffer[32];

    
    //Fill org image buffer with white
    for(int i = 0; i < 800*600*3; i++ ){ imgorgbuffer[i] = 128;}

    //Fill org segmented image buffer with white
    for(int i = 0; i < 800*600*3; i++ ){ imgorgsegmented[i] = 128;}    


    //initializing subimages with white
    for(int j = 0; j < SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGTH*3; j+=3 ){
		subimages1[j] = 255;
		subimages1[j+1] = 255;
		subimages1[j+2] = 255;
    }



    //Init  Gtk
    gtk_init (&argc, &argv);


    orgimage_widget = gtk_image_new();
    orgpixbuf = gdk_pixbuf_new_from_data (imgorgbuffer, GDK_COLORSPACE_RGB,
		                                  FALSE, 8, 800, 600, 800*3, NULL, NULL);

    orgimage_widgetSegmented = gtk_image_new();
    orgpixbufSegmented = gdk_pixbuf_new_from_data (imgorgsegmented, GDK_COLORSPACE_RGB,
		                                  FALSE, 8, 800, 600, 800*3, NULL, NULL);
	
	
	
    for (int kk=0; kk<TOTAL_SUB_IMAGES;kk++){
    	allSubimages[kk].smallimage_widget = gtk_image_new();
    	allSubimages[kk].pixbufsmall = gdk_pixbuf_new_from_data (subimages1, GDK_COLORSPACE_RGB,
    			                                FALSE, 8, SUB_SAMPLING_WIDTH, SUB_SAMPLING_HEIGTH,
    			                                SUB_SAMPLING_WIDTH*3, NULL, NULL);

    	gtk_image_set_from_pixbuf(GTK_IMAGE( allSubimages[kk].smallimage_widget ), allSubimages[kk].pixbufsmall);
    	g_object_unref(allSubimages[kk].pixbufsmall);


    	sprintf (buffer, "SubImage (%d)\n", kk+1);
    	allSubimages[kk].label = gtk_label_new (buffer);
        gtk_misc_set_alignment (GTK_MISC (allSubimages[kk].label), 0, 0);

    }

    /* Create our window */
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

    g_signal_connect (window, "delete-event",
		      G_CALLBACK (delete_event), NULL);
    gtk_window_set_default_size(GTK_WINDOW(window), 250, 250);
    gtk_window_set_title(GTK_WINDOW(window), "Soccer Vision Gui");
    gtk_container_set_border_width(GTK_CONTAINER(window), 0);


    box1 = gtk_hbox_new (FALSE, 0);


    GtkWidget *boxv1;
    GtkWidget *boxv2;

    boxv1 = gtk_vbox_new (TRUE,0);
    gtk_widget_set_size_request (boxv1, 820, 720);

    boxv2 = gtk_vbox_new (TRUE,0);
    gtk_widget_set_size_request (boxv2, 220, 720);

    separator = gtk_vseparator_new ();


    label = gtk_label_new ("Original image");
	labelSegmented = gtk_label_new ("Image after color segmentation");


//     gtk_misc_set_alignment (GTK_MISC (label), 0, 0);
//     gtk_box_pack_start (GTK_BOX(boxv1), label, FALSE, TRUE, 0);
//     gtk_widget_show (label);
// 
// 	gtk_image_set_from_pixbuf(GTK_IMAGE( orgimage_widget ), orgpixbuf);
// 	gtk_widget_queue_draw(orgimage_widget);
// 	gtk_widget_show (orgimage_widget);
// 	gtk_box_pack_start (GTK_BOX(boxv1), orgimage_widget, FALSE, TRUE, 0);
// 	g_object_unref(orgpixbuf);

	
    GtkWidget *scrolled_window_0;
    scrolled_window_0 = gtk_scrolled_window_new (NULL, NULL);
    
    gtk_container_set_border_width (GTK_CONTAINER (scrolled_window_0), 0);
    
    gtk_scrolled_window_set_policy (GTK_SCROLLED_WINDOW (scrolled_window_0),
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_ALWAYS);
    gtk_box_pack_start (GTK_BOX(boxv1), scrolled_window_0, TRUE, TRUE, 0);

    gtk_widget_show (scrolled_window_0);	 

    GtkWidget *table_0;
    table_0 = gtk_table_new (1, 8, FALSE);
    
    /* set the spacing to 10 on x and 10 on y */
    gtk_table_set_row_spacings (GTK_TABLE (table_0), 0);
    gtk_table_set_col_spacings (GTK_TABLE (table_0), 0);
    
    //pack the table into the scrolled window
   gtk_scrolled_window_add_with_viewport (
                   GTK_SCROLLED_WINDOW (scrolled_window_0), table_0);
    gtk_widget_show (table_0);
	

	
		gtk_image_set_from_pixbuf(GTK_IMAGE( orgimage_widget ), orgpixbuf);
		gtk_widget_queue_draw(orgimage_widget);
		//gtk_widget_show (orgimage_widget);	

		gtk_image_set_from_pixbuf(GTK_IMAGE( orgimage_widgetSegmented ), orgpixbufSegmented);
		gtk_widget_queue_draw(orgimage_widgetSegmented);
		//gtk_widget_show (orgimage_widgetSegmented);	
				
		//Label
         gtk_table_attach_defaults (GTK_TABLE (table_0), label,0, 1, 0, 1);
  	     gtk_widget_show (label);

  	     //Image
          gtk_table_attach_defaults (GTK_TABLE (table_0),orgimage_widget ,0,1,1,2);
          gtk_widget_show (orgimage_widget);

		//Label
         gtk_table_attach_defaults (GTK_TABLE (table_0), labelSegmented ,0, 1, 2, 3);
  	     gtk_widget_show (labelSegmented);

  	     //Image
          gtk_table_attach_defaults (GTK_TABLE (table_0),orgimage_widgetSegmented ,0,1,3,4);
          gtk_widget_show (orgimage_widgetSegmented);

	
	
	
	
    GtkWidget *scrolled_window;
    scrolled_window = gtk_scrolled_window_new (NULL, NULL);
    
    gtk_container_set_border_width (GTK_CONTAINER (scrolled_window), 0);
    
    gtk_scrolled_window_set_policy (GTK_SCROLLED_WINDOW (scrolled_window),
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_ALWAYS);

     gtk_box_pack_start (GTK_BOX(boxv2), scrolled_window, TRUE, TRUE, 0);
 
    gtk_widget_show (scrolled_window);
	


    GtkWidget *table;
    table = gtk_table_new (1, 90, FALSE);
    
    /* set the spacing to 10 on x and 10 on y */
    gtk_table_set_row_spacings (GTK_TABLE (table), 0);
    gtk_table_set_col_spacings (GTK_TABLE (table), 0);
    
    //pack the table into the scrolled window
   gtk_scrolled_window_add_with_viewport (
                   GTK_SCROLLED_WINDOW (scrolled_window), table);
    gtk_widget_show (table);
    


	
	
    //scrolled window area
    //Including all the images in a table
    for (int i = 0; i < 1; i++)
       for (int j = 0; j < TOTAL_SUB_IMAGES; j+=1) {


    	 //Image label
         gtk_table_attach_defaults (GTK_TABLE (table), allSubimages[j].label,i, i+1, (j*3), (j*3)+1);
  	     gtk_widget_show (allSubimages[j].label);

  	     //Image
          gtk_table_attach_defaults (GTK_TABLE (table), allSubimages[j].smallimage_widget, i, i+1, (j*3)+1, (j*3)+2);
          gtk_widget_show (allSubimages[j].smallimage_widget);


         //separator
         separator2 = gtk_hseparator_new ();
         gtk_table_attach_defaults (GTK_TABLE (table), separator2, i, i+1, (j*3)+2, (j*3)+3);
	     gtk_widget_show (separator2);
	 
       }



    //packing and showing all widgets
	gtk_box_pack_start (GTK_BOX (box1), boxv1, FALSE, FALSE, 0);
	gtk_widget_show (boxv1);

	gtk_box_pack_start (GTK_BOX (box1), separator, FALSE, TRUE, 5);
	gtk_widget_show (separator);

	gtk_box_pack_start (GTK_BOX (box1), boxv2, FALSE, FALSE, 0);
	gtk_widget_show (boxv2);


    gtk_container_add (GTK_CONTAINER (window), box1);    
    gtk_widget_show (box1);
    gtk_widget_show (window);
    

    ///main Loop ///
    while(MAINLOOP_CONTROL && nopVisonNode.ok()){

    	ros::spinOnce();


        orgpixbuf = gdk_pixbuf_new_from_data (imgorgbuffer, GDK_COLORSPACE_RGB,
    		                                  FALSE, 8, 800, 600, 800*3, NULL, NULL);
    	gtk_image_set_from_pixbuf(GTK_IMAGE( orgimage_widget ), orgpixbuf);
    	gtk_widget_show (orgimage_widget);
    	g_object_unref(orgpixbuf);


        orgpixbufSegmented = gdk_pixbuf_new_from_data (imgorgsegmented, GDK_COLORSPACE_RGB,
    		                                  FALSE, 8, 800, 600, 800*3, NULL, NULL);
    	gtk_image_set_from_pixbuf(GTK_IMAGE( orgimage_widgetSegmented ), orgpixbufSegmented);
    	gtk_widget_show (orgimage_widgetSegmented);
    	g_object_unref(orgpixbufSegmented);
		

    	    for (int kk=0; kk<TOTAL_SUB_IMAGES;kk++){

    	    	allSubimages[kk].pixbufsmall = gdk_pixbuf_new_from_data (allSubimages[kk].img, GDK_COLORSPACE_RGB,
    	    			                                FALSE, 8, SUB_SAMPLING_WIDTH, SUB_SAMPLING_HEIGTH,
    	    			                                SUB_SAMPLING_WIDTH*3, NULL, NULL);

    	    	gtk_image_set_from_pixbuf(GTK_IMAGE( allSubimages[kk].smallimage_widget ), allSubimages[kk].pixbufsmall);
    	    	g_object_unref(allSubimages[kk].pixbufsmall);
    	    	gtk_widget_show (allSubimages[kk].smallimage_widget);
    	    }


       	 while (gtk_events_pending())
       	    gtk_main_iteration();




    }

    //Exit the application
    //gtk_main_quit ();
    gtk_main_iteration_do(0);


    
    return 0;
}
