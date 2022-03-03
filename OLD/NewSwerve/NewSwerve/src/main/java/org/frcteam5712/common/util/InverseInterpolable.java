package org.frcteam5712.common.util;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}
