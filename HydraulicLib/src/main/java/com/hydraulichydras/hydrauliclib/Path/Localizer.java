package com.hydraulichydras.hydrauliclib.Path;

import com.hydraulichydras.hydrauliclib.Geometry.Pose;

public interface Localizer {

    void periodic();
    Pose getPos();
    void setPos(Pose pose);
}
