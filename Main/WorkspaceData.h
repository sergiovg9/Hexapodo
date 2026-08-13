#pragma once

struct LineaTrabajo {
  float xi;
  float yi;
  float xf;
  float yf;
  float longitud;
};

extern const int NUM_ALTURAS;
extern const int NUM_ANGULOS;

extern const float alturas[];
extern const float angulos[];

extern const LineaTrabajo workspace[][180];