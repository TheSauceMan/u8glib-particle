#include "Sauce-U8glib/Sauce-U8glib.h"

SYSTEM_MODE(AUTOMATIC);

#define DISP_A0 D0
#define DISP_RST D1
#define DISP_CS D2

U8GLIB_SSD1331_96X64_4X_332 u8g(DISP_CS, DISP_A0, DISP_RST);

char deviceName[32];

void handler(const char *topic, const char *data)
{
	if (strcmp(topic, "spark/device/name") == 0)
	{
		strncpy(deviceName, data, sizeof(deviceName));
		deviceName[sizeof(deviceName)-1] = 0;
	}
}

void setColor(U8GLIB *pU8GLIB, uint8_t r, uint8_t g, uint8_t b)
{
	if ( pU8GLIB->getMode() == U8G_MODE_HICOLOR )
	{
		pU8GLIB->setHiColorByRGB(r, g, b);
	}
	else
	{
		uint8_t color332 = (r & 0xe0) | ((g & 0xe0) >> 3) | ((b & 0xc0) >> 6);
		pU8GLIB->setColorIndex(color332);
	}
}

void setup()
{
	Particle.subscribe("spark/", handler);
	Particle.publish("spark/device/name");

	setColor(&u8g, 255,255,255);
}

void draw(U8GLIB *pU8GLIB)
{
	// Write devic name
	setColor(pU8GLIB, 255, 255, 255);

	pU8GLIB->setFont(u8g_font_unifont);
	pU8GLIB->drawStr( 0, 22, "Device Name:");

	setColor(pU8GLIB, 255, 255, 0);
	pU8GLIB->drawStr( 0, 33, deviceName);

	// Here's some small scrolling colored bars
	static int frame = 0;
	int x = 0;

	for (int element = 0; element < 3; element++)
	{
		int r = (element == 0) ? 1 : 0;
		int g = (element == 1) ? 1 : 0;
		int b = (element == 2) ? 1 : 0;

		for (int col = 0; col < 30; col++)
		{
			int color = col * 255 / 29;
			setColor(pU8GLIB, color * r, color * g, color * b);
			pU8GLIB->drawVLine((x + frame) % 90, 43, 10);
			x++;
		}
	}

	frame++;
}

void loop()
{
	// U8glib needs something called a draw loop
	// It's used because sometimes a system doesn't have enough
	// memory to draw the whole screen at once, so it repeats over
	// and over drawing little chunks at once.
	u8g.firstPage();
	do
	{
		draw(&u8g);
	}
	while (u8g.nextPage());
}
