/**
 * Arduino - Color Recognition Sensor
 *
 * ColorRecognitionTCS230.h
 *
 * The abstract class for the Color Recognition TCS230 sensor.
 *
 * Thanks https://github.com/combs for your contribution to prevent overflow
 * when returning the frequencies.
 *
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __ARDUINO_DRIVER_COLOR_RECOGNITION_TCS230_CPP__
#define __ARDUINO_DRIVER_COLOR_RECOGNITION_TCS230_CPP__ 1

#include "ColorRecognitionTCS230.h"
#include <Arduino.h>
#include <TimerOne.h>

ColorRecognitionTCS230 ColorRecognitionTCS230::instance;

void ColorRecognitionTCS230::initialize(uint8_t outPin, uint8_t s2Pin, uint8_t s3Pin) {
    this->s2Pin = s2Pin;
    this->s3Pin = s3Pin;
    this->outPin = outPin;
    this->currentFilter = CLEAR_FILTER;
    pinMode(s2Pin, OUTPUT);
    pinMode(s3Pin, OUTPUT);
    pinMode(outPin, INPUT);

    instance.count = 0;
    Timer1.initialize();
    Timer1.attachInterrupt(ColorRecognitionTCS230::timerInterruptHandler);
    attachInterrupt((outPin - 2), ColorRecognitionTCS230::externalInterruptHandler, RISING);
}

void ColorRecognitionTCS230::externalInterruptHandler() {
    instance.count++;
}

void ColorRecognitionTCS230::timerInterruptHandler() {
    switch (instance.currentFilter) {
    case RED_FILTER:
        instance.lastFrequencies[RED] = instance.count;
	instance.whiteBalanceFrequencies[RED] = max(instance.whiteBalanceFrequencies[RED],
						  instance.count);
	instance.blackBalanceFrequencies[RED] = min(instance.blackBalanceFrequencies[RED],
						  instance.count);

        setFilter(GREEN_FILTER);
        break;

    case GREEN_FILTER:
        instance.lastFrequencies[GREEN] = instance.count;
	instance.whiteBalanceFrequencies[GREEN] = max(instance.whiteBalanceFrequencies[GREEN],
						  instance.count);
	instance.blackBalanceFrequencies[GREEN] = min(instance.blackBalanceFrequencies[GREEN],
						  instance.count);

        setFilter(BLUE_FILTER);
        break;

    case BLUE_FILTER:
        instance.lastFrequencies[BLUE] = instance.count;
	instance.whiteBalanceFrequencies[BLUE] = max(instance.whiteBalanceFrequencies[BLUE],
						  instance.count);
	instance.blackBalanceFrequencies[BLUE] = min(instance.blackBalanceFrequencies[BLUE],
						  instance.count);

        setFilter(RED_FILTER);
        break;

    case CLEAR_FILTER:
        setFilter(RED_FILTER);
        break;
    }
    instance.count = 0;
    Timer1.setPeriod(COLOR_PERIOD_US);
}

void ColorRecognitionTCS230::getFreqs(uint32_t freqs[3]) {
  freqs[RED] = instance.lastFrequencies[RED];
  freqs[GREEN] = instance.lastFrequencies[GREEN];
  freqs[BLUE] = instance.lastFrequencies[BLUE];
}

void ColorRecognitionTCS230::getWhiteBal(uint32_t bal[3]) {
  bal[RED] = instance.whiteBalanceFrequencies[RED];
  bal[GREEN] = instance.whiteBalanceFrequencies[GREEN];
  bal[BLUE] = instance.whiteBalanceFrequencies[BLUE];
}

void ColorRecognitionTCS230::getBlackBal(uint32_t bal[3]) {
  bal[RED] = instance.blackBalanceFrequencies[RED];
  bal[GREEN] = instance.blackBalanceFrequencies[GREEN];
  bal[BLUE] = instance.blackBalanceFrequencies[BLUE];
}

void ColorRecognitionTCS230::setWhiteBal(uint32_t bal[3]) {
  instance.whiteBalanceFrequencies[RED] = bal[RED];
  instance.whiteBalanceFrequencies[GREEN] = bal[GREEN];
  instance.whiteBalanceFrequencies[BLUE] = bal[BLUE];
}

void ColorRecognitionTCS230::setBlackBal(uint32_t bal[3]) {
  instance.blackBalanceFrequencies[RED] = bal[RED];
  instance.blackBalanceFrequencies[GREEN] = bal[GREEN];
  instance.blackBalanceFrequencies[BLUE] = bal[BLUE];
}

uint32_t ColorRecognitionTCS230::getCount() {
  return instance.count;
}

uint8_t ColorRecognitionTCS230::getRed() {
    return (uint8_t) map(lastFrequencies[RED],
			 blackBalanceFrequencies[RED], whiteBalanceFrequencies[RED],
			 0, 255);
}

uint8_t ColorRecognitionTCS230::getGreen() {
    return (uint8_t) map(lastFrequencies[GREEN],
			 blackBalanceFrequencies[GREEN], whiteBalanceFrequencies[GREEN],
			 0, 255);
}

uint8_t ColorRecognitionTCS230::getBlue() {
    return (uint8_t) map(lastFrequencies[BLUE],
			 blackBalanceFrequencies[BLUE], whiteBalanceFrequencies[BLUE],
			 0, 255);
}

bool ColorRecognitionTCS230::fillRGB(uint8_t buf[3]) {
    buf[RED] = getRed();
    buf[GREEN] = getGreen();
    buf[BLUE] = getBlue();
    return true;
}

void ColorRecognitionTCS230::setFilter(Filter filter) {
  uint8_t s2;
  uint8_t s3;
  instance.currentFilter = filter;

  switch (filter) {
  case RED_FILTER:
    s2 = LOW;
    s3 = LOW;
    break;

  case GREEN_FILTER:
    s2 = HIGH;
    s3 = LOW;
    break;

  case BLUE_FILTER:
    s2 = LOW;
    s3 = HIGH;
    break;

  case CLEAR_FILTER:
    s2 = HIGH;
    s3 = HIGH;
    break;
  }
  digitalWrite(instance.s2Pin, s2);
  digitalWrite(instance.s3Pin, s3);
}

#endif /* __ARDUINO_DRIVER_COLOR_RECOGNITION_TCS230_CPP__ */
