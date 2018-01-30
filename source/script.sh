 tar Jxf mesa-10.5.4.tar.xz
 cd mesa-10.5.4/
 
 autoreconf -fi
 
 ./configure \
   CXXFLAGS="-O2 -g -DDEFAULT_SOFTWARE_DEPTH_BITS=31" \
   CFLAGS="-O2 -g -DDEFAULT_SOFTWARE_DEPTH_BITS=31" \
   --disable-xvmc \
   --disable-glx \
   --disable-dri \
   --with-dri-drivers="" \
   --with-gallium-drivers="" \
   --disable-shared-glapi \
   --disable-egl \
   --with-egl-platforms="" \
   --enable-osmesa \
   --enable-gallium-llvm=no \
   --prefix=/usr/local/mesa/10.2.2/classic
 make -j4
 # optional
 make check
