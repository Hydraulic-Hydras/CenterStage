package com.hydraulichydras.hydrauliclib.Localization;

import com.hydraulichydras.hydrauliclib.Geometry.Pose;

public interface Localizer {

    void periodic();
    Pose getPos();
    void setPos(Pose pose);

}
