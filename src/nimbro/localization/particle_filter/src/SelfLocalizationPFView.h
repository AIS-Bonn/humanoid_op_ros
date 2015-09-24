#pragma once

class SelfLocalizationPF;

class SelfLocalizationPFView
{
	const float scale;
	static const int centerX = 280;
	static const int centerY = 220;

public:

	SelfLocalizationPFView( SelfLocalizationPF * selfLocalization );
	~SelfLocalizationPFView( ) { }

	inline void
	setLeftClick( int x, int y )
	{
		m_clickx = x;
		m_clicky = y;
		m_selfLocalization->setPose(
			Vec3f(m_clicky-centerY,m_clickx-centerX,0)/scale,
			Vec3f(0.2,0.2,2*M_PI));
	}

	void draw( HDC hDC );


protected:

	int m_clickx, m_clicky;

	SelfLocalizationPF * m_selfLocalization;

	HBRUSH transparentBrush;
	HBRUSH yellowBrush;
	HBRUSH redBrush;
	HBRUSH blueBrush;
	HBRUSH greenBrush;
	HBRUSH orangeBrush;
	HBRUSH grayBrush;
	HBRUSH cyanBrush;
	HBRUSH magentaBrush;

	HPEN thickPen;
	HPEN redPen;
	HPEN greenPen;
	HPEN grayPen;
	HPEN thickGrayPen;
	HPEN thinPen;
	HPEN dashedPen;


};
