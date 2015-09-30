//HSVPresenter.cpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/Tools/HSVPresenter.hpp>

bool HSVPresenter::Update()
{
	ros::Time now=ros::Time::now();

	activeIndex = -1;

	int changed = -1;

	for (int i = 0; i < COLORED_OBJECT_COUNT; i++)
	{
		if (lastActiveRange[i] != currecntRanges[i].active->get())
		{
			changed = i;
		}
	}

	for (int i = 0; i < COLORED_OBJECT_COUNT; i++)
	{
		if (changed >= 0 && changed != i)
		{
			currecntRanges[i].active->set(false);
		}

		if (currecntRanges[i].active->get())
		{
			activeIndex = i;
		}
		lastActiveRange[i] = currecntRanges[i].active->get();
	}
	return true;
}

void HSVPresenter::Publish()
{
	if (activeIndex < 0 || !rangeImg_pub.thereAreListeners())
	{
		return;
	}
	rangeImg = Mat::zeros(rangeImgHeight, rangeImgWidth, CV_8UC3);
	for (int j = 0; j < rangeImgHeight; j++)

	{
		uchar* rows = rangeImg.ptr(j);
		for (int i = 0; i < rangeImgWidth; i++)
		{
			uint16_t h = 0;
			if (currecntRanges[activeIndex].h0->get() <= currecntRanges[activeIndex].h1->get())
			{
				h = currecntRanges[activeIndex].h0->get()
						+ (((currecntRanges[activeIndex].h1->get()
								- currecntRanges[activeIndex].h0->get())
								/ (float) rangeImgHeight) * j);
			}
			else
			{
				int len = currecntRanges[activeIndex].h1->get() + 180
						- currecntRanges[activeIndex].h0->get();
				if (len == 0)
				{
					h = 0;
				}
				else
				{
					double coef = (double) rangeImgHeight / len;
					double bound = (180 - currecntRanges[activeIndex].h0->get()) * coef;
					if (j > bound)
					{
						h = (uint16_t) (0
								+ (currecntRanges[activeIndex].h1->get()
										/ (rangeImgHeight - bound))
										* (j - bound));
					}
					else
					{
						h = (uint16_t) (currecntRanges[activeIndex].h0->get()
								+ ((180 - currecntRanges[activeIndex].h0->get()) / bound)
										* j);
					}

				}

			}
			int s = currecntRanges[activeIndex].s0->get()
					+ (((currecntRanges[activeIndex].s1->get()
							- currecntRanges[activeIndex].s0->get())
							/ (float) rangeImgWidth) * i);
			int v = currecntRanges[activeIndex].v0->get()
					+ (((currecntRanges[activeIndex].v1->get()
							- currecntRanges[activeIndex].v0->get())
							/ (float) rangeImgWidth) * i);

			uchar* pix = (rows + i * 3);
			pix[0] = (uint8_t) (h > 180 ? 180 : h);
			pix[1] = (uint8_t) (s > 255 ? 255 : s);
			pix[2] = (uint8_t) (v > 255 ? 255 : v);
		}

	}
	rangeImg_pub.publish(rangeImg, MatPublisher::hsv);
}
