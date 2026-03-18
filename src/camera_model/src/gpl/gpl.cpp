#include "cameraModelCal/gpl/gpl.h"

#include <set>
#ifdef _WIN32
#include <winsock.h>
#else
#include <time.h>
#endif

#ifdef __APPLE__
#include <mach/mach_time.h>
#define ORWL_NANO (+1.0E-9)
#define ORWL_GIGA UINT64_C(1000000000)

static double orwl_timebase = 0.0;
static uint64_t orwl_timestart = 0;

struct timespec orwl_gettime(void) {
  // be more careful in a multithreaded environement
  if (!orwl_timestart) {
    mach_timebase_info_data_t tb = { 0 };
    mach_timebase_info(&tb);
    orwl_timebase = tb.numer;
    orwl_timebase /= tb.denom;
    orwl_timestart = mach_absolute_time();
  }
  struct timespec t;
  double diff = (mach_absolute_time() - orwl_timestart) * orwl_timebase;
  t.tv_sec = diff * ORWL_NANO;
  t.tv_nsec = diff - (t.tv_sec * ORWL_GIGA);
  return t;
}
#endif // __APPLE__

const double WGS84_A = 6378137.0;
const double WGS84_ECCSQ = 0.00669437999013;

// Windows lacks fminf
#ifndef fminf
#define fminf(x, y) (((x) < (y)) ? (x) : (y))
#endif

namespace cameraModelCal
{

double hypot3(double x,double y,double z)
{
    return sqrt(square(x)+square(y)+square(z));
}

float hypot3f(float x,float y,float z)
{
    return sqrtf(square(x)+square(y)+square(z));
}

double d2r(double deg)
{
    return deg*M_PI/180.0;
}

float d2r(float deg)
{
    return deg*M_PI/180.0f;
}

double r2d(double rad)
{
    return rad*180.0/M_PI;
}

float r2d(float rad)
{
    return rad*180.0f/M_PI;
}

double sinc(double theta)
{
    if (fabs(theta) < 1e-6)
        return 1.0;
    else
        return sin(theta)/theta;
}

#ifdef _WIN32
#include <sys/timeb.h>
#include <sys/types.h>
#include <winsock.h>
LARGE_INTEGER
getFILETIMEoffset()
{
    SYSTEMTIME s;
    FILETIME f;
    LARGE_INTEGER t;

    s.wYear = 1970;
    s.wMonth = 1;
    s.wDay = 1;
    s.wHour = 0;
    s.wMinute = 0;
    s.wSecond = 0;
    s.wMilliseconds = 0;
    SystemTimeToFileTime(&s, &f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
    return (t);
}

int
clock_gettime(int X, struct timespec *tp)
{
    LARGE_INTEGER           t;
    FILETIME            f;
    double                  microseconds;
    static LARGE_INTEGER    offset;
    static double           frequencyToMicroseconds;
    static int              initialized = 0;
    static BOOL             usePerformanceCounter = 0;

    if (!initialized) {
        LARGE_INTEGER performanceFrequency;
        initialized = 1;
        usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
        if (usePerformanceCounter) {
            QueryPerformanceCounter(&offset);
            frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
        } else {
            offset = getFILETIMEoffset();
            frequencyToMicroseconds = 10.;
        }
    }
    if (usePerformanceCounter) QueryPerformanceCounter(&t);
    else {
        GetSystemTimeAsFileTime(&f);
        t.QuadPart = f.dwHighDateTime;
        t.QuadPart <<= 32;
        t.QuadPart |= f.dwLowDateTime;
    }

    t.QuadPart -= offset.QuadPart;
    microseconds = (double)t.QuadPart / frequencyToMicroseconds;
    t.QuadPart = microseconds;
    tp->tv_sec = t.QuadPart / 1000000;
    tp->tv_nsec = (t.QuadPart % 1000000) * 1000;
    return (0);
}
#endif

unsigned long long timeInMicroseconds(void)
{
	 struct timespec tp;
#ifdef __APPLE__
     tp = orwl_gettime();
#else
	 clock_gettime(CLOCK_REALTIME, &tp);
#endif

	 return tp.tv_sec * 1000000 + tp.tv_nsec / 1000;
}

double timeInSeconds(void)
{
    struct timespec tp;
#ifdef __APPLE__
     tp = orwl_gettime();
#else
	 clock_gettime(CLOCK_REALTIME, &tp);
#endif

	 return static_cast<double>(tp.tv_sec) +
			 static_cast<double>(tp.tv_nsec) / 1000000000.0;
}

float colormapAutumn[128][3] =
{
		{1.0f,0.f,0.f},
		{1.0f,0.007874f,0.f},
		{1.0f,0.015748f,0.f},
		{1.0f,0.023622f,0.f},
		{1.0f,0.031496f,0.f},
		{1.0f,0.03937f,0.f},
		{1.0f,0.047244f,0.f},
		{1.0f,0.055118f,0.f},
		{1.0f,0.062992f,0.f},
		{1.0f,0.070866f,0.f},
		{1.0f,0.07874f,0.f},
		{1.0f,0.086614f,0.f},
		{1.0f,0.094488f,0.f},
		{1.0f,0.10236f,0.f},
		{1.0f,0.11024f,0.f},
		{1.0f,0.11811f,0.f},
		{1.0f,0.12598f,0.f},
		{1.0f,0.13386f,0.f},
		{1.0f,0.14173f,0.f},
		{1.0f,0.14961f,0.f},
		{1.0f,0.15748f,0.f},
		{1.0f,0.16535f,0.f},
		{1.0f,0.17323f,0.f},
		{1.0f,0.1811f,0.f},
		{1.0f,0.18898f,0.f},
		{1.0f,0.19685f,0.f},
		{1.0f,0.20472f,0.f},
		{1.0f,0.2126f,0.f},
		{1.0f,0.22047f,0.f},
		{1.0f,0.22835f,0.f},
		{1.0f,0.23622f,0.f},
		{1.0f,0.24409f,0.f},
		{1.0f,0.25197f,0.f},
		{1.0f,0.25984f,0.f},
		{1.0f,0.26772f,0.f},
		{1.0f,0.27559f,0.f},
		{1.0f,0.28346f,0.f},
		{1.0f,0.29134f,0.f},
		{1.0f,0.29921f,0.f},
		{1.0f,0.30709f,0.f},
		{1.0f,0.31496f,0.f},
		{1.0f,0.32283f,0.f},
		{1.0f,0.33071f,0.f},
		{1.0f,0.33858f,0.f},
		{1.0f,0.34646f,0.f},
		{1.0f,0.35433f,0.f},
		{1.0f,0.3622f,0.f},
		{1.0f,0.37008f,0.f},
		{1.0f,0.37795f,0.f},
		{1.0f,0.38583f,0.f},
		{1.0f,0.3937f,0.f},
		{1.0f,0.40157f,0.f},
		{1.0f,0.40945f,0.f},
		{1.0f,0.41732f,0.f},
		{1.0f,0.4252f,0.f},
		{1.0f,0.43307f,0.f},
		{1.0f,0.44094f,0.f},
		{1.0f,0.44882f,0.f},
		{1.0f,0.45669f,0.f},
		{1.0f,0.46457f,0.f},
		{1.0f,0.47244f,0.f},
		{1.0f,0.48031f,0.f},
		{1.0f,0.48819f,0.f},
		{1.0f,0.49606f,0.f},
		{1.0f,0.50394f,0.f},
		{1.0f,0.51181f,0.f},
		{1.0f,0.51969f,0.f},
		{1.0f,0.52756f,0.f},
		{1.0f,0.53543f,0.f},
		{1.0f,0.54331f,0.f},
		{1.0f,0.55118f,0.f},
		{1.0f,0.55906f,0.f},
		{1.0f,0.56693f,0.f},
		{1.0f,0.5748f,0.f},
		{1.0f,0.58268f,0.f},
		{1.0f,0.59055f,0.f},
		{1.0f,0.59843f,0.f},
		{1.0f,0.6063f,0.f},
		{1.0f,0.61417f,0.f},
		{1.0f,0.62205f,0.f},
		{1.0f,0.62992f,0.f},
		{1.0f,0.6378f,0.f},
		{1.0f,0.64567f,0.f},
		{1.0f,0.65354f,0.f},
		{1.0f,0.66142f,0.f},
		{1.0f,0.66929f,0.f},
		{1.0f,0.67717f,0.f},
		{1.0f,0.68504f,0.f},
		{1.0f,0.69291f,0.f},
		{1.0f,0.70079f,0.f},
		{1.0f,0.70866f,0.f},
		{1.0f,0.71654f,0.f},
		{1.0f,0.72441f,0.f},
		{1.0f,0.73228f,0.f},
		{1.0f,0.74016f,0.f},
		{1.0f,0.74803f,0.f},
		{1.0f,0.75591f,0.f},
		{1.0f,0.76378f,0.f},
		{1.0f,0.77165f,0.f},
		{1.0f,0.77953f,0.f},
		{1.0f,0.7874f,0.f},
		{1.0f,0.79528f,0.f},
		{1.0f,0.80315f,0.f},
		{1.0f,0.81102f,0.f},
		{1.0f,0.8189f,0.f},
		{1.0f,0.82677f,0.f},
		{1.0f,0.83465f,0.f},
		{1.0f,0.84252f,0.f},
		{1.0f,0.85039f,0.f},
		{1.0f,0.85827f,0.f},
		{1.0f,0.86614f,0.f},
		{1.0f,0.87402f,0.f},
		{1.0f,0.88189f,0.f},
		{1.0f,0.88976f,0.f},
		{1.0f,0.89764f,0.f},
		{1.0f,0.90551f,0.f},
		{1.0f,0.91339f,0.f},
		{1.0f,0.92126f,0.f},
		{1.0f,0.92913f,0.f},
		{1.0f,0.93701f,0.f},
		{1.0f,0.94488f,0.f},
		{1.0f,0.95276f,0.f},
		{1.0f,0.96063f,0.f},
		{1.0f,0.9685f,0.f},
		{1.0f,0.97638f,0.f},
		{1.0f,0.98425f,0.f},
		{1.0f,0.99213f,0.f},
		{1.0f,1.0f,0.0f}
};


float colormapJet[128][3] =
{
		{0.0f,0.0f,0.53125f},
		{0.0f,0.0f,0.5625f},
		{0.0f,0.0f,0.59375f},
		{0.0f,0.0f,0.625f},
		{0.0f,0.0f,0.65625f},
		{0.0f,0.0f,0.6875f},
		{0.0f,0.0f,0.71875f},
		{0.0f,0.0f,0.75f},
		{0.0f,0.0f,0.78125f},
		{0.0f,0.0f,0.8125f},
		{0.0f,0.0f,0.84375f},
		{0.0f,0.0f,0.875f},
		{0.0f,0.0f,0.90625f},
		{0.0f,0.0f,0.9375f},
		{0.0f,0.0f,0.96875f},
		{0.0f,0.0f,1.0f},
		{0.0f,0.03125f,1.0f},
		{0.0f,0.0625f,1.0f},
		{0.0f,0.09375f,1.0f},
		{0.0f,0.125f,1.0f},
		{0.0f,0.15625f,1.0f},
		{0.0f,0.1875f,1.0f},
		{0.0f,0.21875f,1.0f},
		{0.0f,0.25f,1.0f},
		{0.0f,0.28125f,1.0f},
		{0.0f,0.3125f,1.0f},
		{0.0f,0.34375f,1.0f},
		{0.0f,0.375f,1.0f},
		{0.0f,0.40625f,1.0f},
		{0.0f,0.4375f,1.0f},
		{0.0f,0.46875f,1.0f},
		{0.0f,0.5f,1.0f},
		{0.0f,0.53125f,1.0f},
		{0.0f,0.5625f,1.0f},
		{0.0f,0.59375f,1.0f},
		{0.0f,0.625f,1.0f},
		{0.0f,0.65625f,1.0f},
		{0.0f,0.6875f,1.0f},
		{0.0f,0.71875f,1.0f},
		{0.0f,0.75f,1.0f},
		{0.0f,0.78125f,1.0f},
		{0.0f,0.8125f,1.0f},
		{0.0f,0.84375f,1.0f},
		{0.0f,0.875f,1.0f},
		{0.0f,0.90625f,1.0f},
		{0.0f,0.9375f,1.0f},
		{0.0f,0.96875f,1.0f},
		{0.0f,1.0f,1.0f},
		{0.03125f,1.0f,0.96875f},
		{0.0625f,1.0f,0.9375f},
		{0.09375f,1.0f,0.90625f},
		{0.125f,1.0f,0.875f},
		{0.15625f,1.0f,0.84375f},
		{0.1875f,1.0f,0.8125f},
		{0.21875f,1.0f,0.78125f},
		{0.25f,1.0f,0.75f},
		{0.28125f,1.0f,0.71875f},
		{0.3125f,1.0f,0.6875f},
		{0.34375f,1.0f,0.65625f},
		{0.375f,1.0f,0.625f},
		{0.40625f,1.0f,0.59375f},
		{0.4375f,1.0f,0.5625f},
		{0.46875f,1.0f,0.53125f},
		{0.5f,1.0f,0.5f},
		{0.53125f,1.0f,0.46875f},
		{0.5625f,1.0f,0.4375f},
		{0.59375f,1.0f,0.40625f},
		{0.625f,1.0f,0.375f},
		{0.65625f,1.0f,0.34375f},
		{0.6875f,1.0f,0.3125f},
		{0.71875f,1.0f,0.28125f},
		{0.75f,1.0f,0.25f},
		{0.78125f,1.0f,0.21875f},
		{0.8125f,1.0f,0.1875f},
		{0.84375f,1.0f,0.15625f},
		{0.875f,1.0f,0.125f},
		{0.90625f,1.0f,0.09375f},
		{0.9375f,1.0f,0.0625f},
		{0.96875f,1.0f,0.03125f},
		{1.0f,1.0f,0.0f},
		{1.0f,0.96875f,0.0f},
		{1.0f,0.9375f,0.0f},
		{1.0f,0.90625f,0.0f},
		{1.0f,0.875f,0.0f},
		{1.0f,0.84375f,0.0f},
		{1.0f,0.8125f,0.0f},
		{1.0f,0.78125f,0.0f},
		{1.0f,0.75f,0.0f},
		{1.0f,0.71875f,0.0f},
		{1.0f,0.6875f,0.0f},
		{1.0f,0.65625f,0.0f},
		{1.0f,0.625f,0.0f},
		{1.0f,0.59375f,0.0f},
		{1.0f,0.5625f,0.0f},
		{1.0f,0.53125f,0.0f},
		{1.0f,0.5f,0.0f},
		{1.0f,0.46875f,0.0f},
		{1.0f,0.4375f,0.0f},
		{1.0f,0.40625f,0.0f},
		{1.0f,0.375f,0.0f},
		{1.0f,0.34375f,0.0f},
		{1.0f,0.3125f,0.0f},
		{1.0f,0.28125f,0.0f},
		{1.0f,0.25f,0.0f},
		{1.0f,0.21875f,0.0f},
		{1.0f,0.1875f,0.0f},
		{1.0f,0.15625f,0.0f},
		{1.0f,0.125f,0.0f},
		{1.0f,0.09375f,0.0f},
		{1.0f,0.0625f,0.0f},
		{1.0f,0.03125f,0.0f},
		{1.0f,0.0f,0.0f},
		{0.96875f,0.0f,0.0f},
		{0.9375f,0.0f,0.0f},
		{0.90625f,0.0f,0.0f},
		{0.875f,0.0f,0.0f},
		{0.84375f,0.0f,0.0f},
		{0.8125f,0.0f,0.0f},
		{0.78125f,0.0f,0.0f},
		{0.75f,0.0f,0.0f},
		{0.71875f,0.0f,0.0f},
		{0.6875f,0.0f,0.0f},
		{0.65625f,0.0f,0.0f},
		{0.625f,0.0f,0.0f},
		{0.59375f,0.0f,0.0f},
		{0.5625f,0.0f,0.0f},
		{0.53125f,0.0f,0.0f},
		{0.5f,0.0f,0.0f}
};

long int timestampDiff(uint64_t t1,uint64_t t2)
{
    if(t2>t1)
    {
        uint64_t d = t2 - t1;

        if (d > std::numeric_limits<long int>::max())
        {
            return std::numeric_limits<long int>::max();
        }
        else
        {
            return d;
        }
    }
    else
    {
        uint64_t d = t1 - t2;

        if (d > std::numeric_limits<long int>::max())
        {
            return std::numeric_limits<long int>::min();
        }
        else
        {
            return - static_cast<long int>(d);
        }
    }
}

void colorDepthImage(cv::Mat& imgDepth,
                    cv::Mat& imgColoredDepth,
                    float minRange,floar maxRange
)
{
    imgColoredDepth = cv::Mat(imgDepth.size(),CV_8UC3);
    for(int i=0;i<imgDepth.rows;++i)
    {
        const float* depth = imgDepth.ptr<float>(i);
        unsigned char* pixel = imgColoredDepth.ptr<unsigned char>(i);
        for(int j=0;j<imgDepth.cols;++j)
        { 
            if(depth[j]!=0)
            {
                int idx=fminf(depth[j]-minRange,maxRange-minRange)*127.0f;
                idx=127-idx;

                pixel[0]=colormapJet[idx][2]*255.0f;
                pixel[1]=colormapJet[idx][1]*255.0f;
                pixel[2]=colormap
            }

            pixel+=3;
        }
    }
}

bool colormap(const std::string& name,unsigned char idx,
float& r,float& g, float& b)
{
    if(name.compare("jet")==0)
    { 
        float* color = colormapJet[idx];
        r=color[0];
        g=color[1];
        b=color[2];

        return true;
    }
    else if(name.compare("autumn")==0)
    {
        float* color = colormapAutumn[idx];
        r=color[0];
        g=color[1];
        b=color[2];

        return true;
    }

    return false;
}

std::vector<cv::Point2i> bresLine(int x0,int y0,int x1,int y1)
{
    std::vector<cv::Point2i> cells;
    
    int dx=std::abs(x1-x0);
    int dy=std::abs(y1-y0);

    int sx=(x0<x1)?1:-1;
    int sy=(y0<y1)?1:-1;
    
    int err=dx-dy;

    while(true)
    {
        cells.push_back(cv::Point2i(x0,y0));
        
        if(x0==x1 && y0==y1)
        {
            break;
        }
        int e2=2*err;
        if (e2>-dy)
        {
            err-=dy;
            x0+=sx;
        }
        if (e2<dx)
        {
            err+=dx;
            y0+=sy;
        }
    }

    return cells;
}

std::vector<cv::Point2i> bresCircle(int x0,int y0,int r)
{
    std::vector<std::vector<bool>> mask(2*r+1);

    for(int i=0;i<2*r+1;++i)
    {
        mask[i].resize(2*r+1,false);
        for (int j = 0; j<2*r+1; ++j)
        {
            mask[i][j]=false;
        }
    }

    int f=1-r;
    int ddF_x=1;
    int ddF_y=-2*r;
    int x=0;
    int y=r;

    std::vector<cv::Point2i> line;

    line=bresLine(x0,y0-r,x0,y0+r);
    for(std::vector<cv::Point2i>::iterator it=line.begin();it!=line.end();++it)
    {
        mask[it->x-x0+r][it->y-y0+r]=true;
    }
    line=bresLine(x0-r,y0,x0+r,y0);
    for(std::vector<cv::Point2i>::iterator it=line.begin();it!=line.end();++it)
    {
        mask[it->x-x0+r][it->y-y0+r]=true;
    }

    while(x<y)
    {
        if(f>=0)
        {//向下走
            y--;
            ddF_y+=2;
            f+=ddF_y;
        }

        //向右走
        x++;
        ddF_x+=2;
        f+=ddF_x;

        line = bresLine(x0 - x, y0 + y, x0 + x, y0 + y);
		for (std::vector<cv::Point2i>::iterator it = line.begin(); it != line.end(); ++it)
		{
			mask[it->x - x0 + r][it->y - y0 + r] = true;
		}

        line=bresLine(x0-x,y0-y,x0+x,y0-y);
        for(std::vector<cv::Point2i>::iterator it=line.begin();it!=line.end();++it)
        {
            mask[it->x-x0+r][it->y-y0+r]=true;
        }

		line = bresLine(x0 - y, y0 + x, x0 + y, y0 + x);
		for (std::vector<cv::Point2i>::iterator it = line.begin(); it != line.end(); ++it)
		{
			mask[it->x - x0 + r][it->y - y0 + r] = true;
		}

		line = bresLine(x0 - y, y0 - x, x0 + y, y0 - x);
		for (std::vector<cv::Point2i>::iterator it = line.begin(); it != line.end(); ++it)
		{
			mask[it->x - x0 + r][it->y - y0 + r] = true;
		}       
    }

    std::vector<cv::Point2i> cells;
    for(int i=0;i<2*r+1;++i)
    {
        for(int j=0;j<2*r+1;++j)
        {
            if(mask[i][j])
            {
                cells.push_back(cv::Point2i(i-r+x0,j-r+y0));
            }
        }
    }

    return cells;
}

}
