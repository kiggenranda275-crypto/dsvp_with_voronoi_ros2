#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

#define INTPOINT IntPoint

/*! A light-weight integer point with fields x,y */
class IntPoint
{
public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}
  int x, y;
};

typedef struct
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
} RGB;

#endif
