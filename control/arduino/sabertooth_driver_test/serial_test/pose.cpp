#include "pose.h"

String pose::toString() {
    int val_size = 8;
    char xbuf[val_size];
    char ybuf[val_size];
    char zbuf[val_size];
    dtostrf(x, val_size-1, 2, xbuf);
    dtostrf(y, val_size-1, 2, ybuf);
    dtostrf(z, val_size-1, 2, zbuf);

    char buffer[val_size*3 + 14];
    sprintf(buffer, "Pose: (%s, %s, %s) ", xbuf, ybuf, zbuf);
    return buffer;
}
