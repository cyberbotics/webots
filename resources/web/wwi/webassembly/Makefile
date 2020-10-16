EMCC = emcc
SOURCES = Color.cpp

EMFLAGS = -s USE_WEBGL2=1 -s FULL_ES3=1 -s WASM=0 -Iglm

FUNCTIONS_TO_EXPORT = -s EXPORTED_FUNCTIONS='["_wr_color_new","_wr_color_is_color","_wr_color_r","_wr_color_g","_wr_color_b"]' -s EXPORTED_RUNTIME_METHODS='["ccall","cwrap"]'


build: $(SOURCES)
	$(EMCC) $(EMFLAGS) $(SOURCES) $(FUNCTIONS_TO_EXPORT) -o ../my_lib.js
