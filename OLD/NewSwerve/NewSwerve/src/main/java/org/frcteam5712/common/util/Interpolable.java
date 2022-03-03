package org.frcteam5712.common.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
