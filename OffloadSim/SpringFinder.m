% Spring Finder
close all; clear; clc;

data = readtable("Spring Data.xlsx");

IPPT = input("Required IPPT: ");
err = input("Acceptable Overhead (%): ");
minLen = input("Min Length (in): ");
maxLen = input("Max Length (in): ");

if maxLen < minLen
    error("Error: Max Length is shorter than Min Length");
end


maxIPPT = IPPT*(1+(err/100));

acceptable_IPPT = data((data.IPPT > IPPT) & (data.IPPT < maxIPPT),:);
acceptable_len = acceptable_IPPT((acceptable_IPPT.Length_in_ >= minLen) & (acceptable_IPPT.Length_in_ <= maxLen),:);

disp(acceptable_len);



















