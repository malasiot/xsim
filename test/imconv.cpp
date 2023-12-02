#include "imconv.hpp"

using namespace xviz;

static void hsv2rgb(float h, QRgb &rgb)
{
    int i ;
    float f, p, q, t, r, g, b ;

    if ( h == 0.0 ) return ;

    // h = 360.0-h ;

    h /= 60.0 ;

    i = (int)h ;
    f = h - i ;
    p = 0  ;
    q = 1-f ;
    t = f ;

    switch (i)
    {
        case 0:
            r = 1 ;
            g = t ;
            b = p ;
            break ;
        case 1:
            r = q ;
            g = 1 ;
            b = p ;
            break ;
        case 2:
            r = p ;
            g = 1 ;
            b = t ;
            break ;
        case 3:
            r = p ;
            g = q ;
            b = 1 ;
            break ;
        case 4:
            r = t ;
            g = p ;
            b = 1 ;
            break ;
        case 5:
            r = 1 ;
            g = p ;
            b = q ;
            break ;
    }

    rgb = qRgb((int)(255.0*r), (int)(255.0*g), (int)(255.0*b)) ;
}

const int nColors = 2 << 12 ;
static QRgb *hsvlut ;


QImage imageToQImage(const xviz::Image &img) {
    uint32_t w = img.width(), h = img.height(), lw ;

    if ( img.type() != ImageType::Raw ) {
        QImage blank(w, h, QImage::Format_RGB888) ;
        blank.fill(0) ;
        return blank ;
    }

    if ( img.format() == ImageFormat::rgb24 ) {
        QImage image((uchar *)img.data(), w, h, w * 3, QImage::Format_RGB888) ;
        return image ;
    }
    else if ( img.format() == ImageFormat::rgba32 )
    {
        QImage image(w, h, QImage::Format_ARGB32) ;

        for( int i=0 ; i<h ; i++ )
        {
            uchar *dst = image.scanLine(i), *src = (uchar *)img.data() + i * w * 4 ;

            for( int j=0 ; j<w ; j++ )
            {
                uchar R = *src++ ;
                uchar G = *src++ ;
                uchar B = *src++ ;
                uchar A = *src++ ;

                *(QRgb *)dst = qRgba(R, G, B, A) ;
                dst += 4 ;
            }
        }

        return image ;
    }
    else if ( img.format() == ImageFormat::gray16 ) {
         int nc = nColors ;
         lw = w * 2 ;

         if ( !hsvlut )
         {
             int c ;
             float h, hmax, hstep ;

             hsvlut = new QRgb [nColors] ;

             hmax = 180 ;
             hstep = hmax/nc ;

             for ( c=0, h=hstep ; c<nc ; c++, h += hstep) hsv2rgb(h, hsvlut[c]) ;
         }

         unsigned short minv, maxv ;
         int i, j ;

         minv = 0xffff ;
         maxv = 0 ;

         const uchar *ppl = img.data() ;
         unsigned short *pp = (unsigned short *)ppl ;

         for ( i=0 ; i<h ; i++, ppl += lw )
             for ( j=0, pp = (unsigned short *)ppl ; j<w ; j++, pp++ )
             {
                 if ( *pp == 0 ) continue ;
                 maxv = qMax(*pp, maxv) ;
                 minv = qMin(*pp, minv) ;
             }

         QImage image(w, h, QImage::Format_RGB32) ;

         for( i=0 ; i<h ; i++ )
         {
             uchar *dst = image.scanLine(i) ;
             const unsigned short *src = (const unsigned short *)img.data() + i * w ;

             for( j=0 ; j<w ; j++ )
             {
                 unsigned short val = *src++ ;

                 if ( val == 0 )
                 {
                     *(QRgb *)dst = Qt::black ;
                     dst += 3 ;
                     *dst++ = 255 ;

                     continue ;
                 }
                 else val = (nc-1)*float((val - minv)/float(maxv - minv)) ;

                 const QRgb &clr = hsvlut[val] ;

                 *(QRgb *)dst = clr ;
                 dst += 3 ;
                 *dst++ = 255 ;
             }
         }

         return image ;

     }


    return QImage() ;
}
