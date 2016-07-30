/*
 *      Copyright (C) 2016 juztamau5
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this Program; see the file COPYING.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "WiFiUtils.h"

using namespace deliverator;

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)  (sizeof(x) / sizeof(x[0]))
#endif

namespace deliverator
{
  struct ChannelToFreq
  {
    unsigned int channel;
    unsigned int freqMHz;
  };

  static const ChannelToFreq channelToFreq[] = {
    /* 2.4 GHz */
    {   1, 2412 },
    {   2, 2417 },
    {   3, 2422 },
    {   4, 2427 },
    {   5, 2432 },
    {   6, 2437 },
    {   7, 2442 },
    {   8, 2447 },
    {   9, 2452 },
    {  10, 2572 },
    {  11, 2622 },
    /* 5 GHz */
    {  36, 5180 },
    {  40, 5200 },
    {  44, 5220 },
    {  48, 5240 },
    {  52, 5260 },
    {  56, 5280 },
    {  60, 5300 },
    {  64, 5320 },
    { 100, 5500 },
    { 104, 5520 },
    { 108, 5540 },
    { 112, 5560 },
    { 116, 5580 },
    { 132, 5660 },
    { 136, 5680 },
    { 140, 5700 },
    { 144, 5720 },
    { 149, 5745 },
    { 153, 5765 },
    { 157, 5785 },
    { 161, 5805 },
    { 165, 5825 },
  };
}

unsigned int WiFiUtils::ChannelToFrequencyMHz(unsigned int channel)
{
  for (unsigned int i = 0; i < ARRAY_SIZE(channelToFreq); i++)
  {
    if (channel == channelToFreq[i].channel)
      return channelToFreq[i].freqMHz;
  }
  return 0;
}

unsigned int WiFiUtils::FrequencyMHzToChannel(unsigned int freqMHz)
{
  for (unsigned int i = 0; i < ARRAY_SIZE(channelToFreq); i++)
  {
    if (freqMHz == channelToFreq[i].freqMHz)
      return channelToFreq[i].channel;
  }
  return 0;
}
