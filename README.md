## I Introduction

Cardiovascular diseases are among the leading causes of mortality worldwide, requiring fast and reliable diagnostic tools. The electrocardiogram (ECG) is a key test for detecting these pathologies, but traditional systems—often bulky and expensive—limit their use outside clinical settings. Meanwhile, organic light-emitting diode (OLED) display technologies offer promising solutions for portable interfaces thanks to their flexibility, low power consumption, and excellent image quality.

This project aims to develop an integrated system combining flexible ECG electrodes with a passive OLED matrix for displaying cardiac signals. Cleanroom fabrication ensures the precision and reliability of the components, while electro-optical characterization optimizes their performance. The electrodes, manufactured on a polyimide substrate with conductive layers of gold and PEDOT:PSS, ensure optimal skin contact. The OLED matrix, meanwhile, is fabricated by vacuum deposition of organic layers (HTL, ETL, EML) and encapsulated for improved stability.

The ultimate goal is to propose a portable, low-cost device for cardiac monitoring that integrates both ECG signal acquisition and its real-time visualization on an OLED interface. This report details the fabrication steps, characterization results, and improvement perspectives for future industrialization.

## II Methods and Techniques Used

The realization of this project relied on the equipment and protocols of an ISO 7 cleanroom, ensuring a controlled environment (< 10,000 particles/m³) for sensitive operations. The methods employed combined advanced techniques of microfabrication and characterization.

<img width="245" height="168" alt="image" src="https://github.com/user-attachments/assets/0e8456e1-fb7f-4d19-8b9d-4a8c9bc05fce" />

###### 1 Fabrication of Flexible ECG Electrodes

The flexible ECG electrodes were fabricated in a cleanroom using advanced microfabrication processes, including photolithography and vacuum evaporation, to ensure device precision and reproducibility.

###### 1.1 Substrate Preparation

Fabrication begins with the use of a flexible polyimide substrate (Kapton, 50 μm thick). The substrate is first firmly fixed onto a glass carrier using Kapton tape to ensure optimal flatness. A quick cleaning with deionized (DI) water and acetone was performed to remove coarse surface contaminants, followed by O₂ plasma activation (Oxford Plasma Etch, 50 sccm O₂, 100 W, 1 min 30 s at 100 mTorr).

###### 1.2 Photolithography

A photoresist layer (S1813) was then deposited by spin coating at 1500 rpm for 50 seconds, followed by a soft bake at 110°C for 1 minute to stabilize the layer. UV exposure was performed using prior alignment with a dedicated mask on the Mask Aligner tool. After hard-contact exposure for 15 seconds, the substrate was developed for 40 seconds in MF26 developer and then thoroughly rinsed with DI water.

<img width="473" height="518" alt="image" src="https://github.com/user-attachments/assets/8b9a9761-097f-4fc7-9fa5-5cd3f26a0b57" />


###### 1.3 Thermal Evaporation

After photolithography, conductive layers were deposited by thermal evaporation under high vacuum. A thin adhesion layer of chromium (5 nm), followed by a conductive gold layer (100–150 nm), was deposited to ensure optimal conductivity. This process was carried out using the Boc Edwards thermal evaporator under a final pressure lower than 1 × 10⁻⁶ Pa, ensuring high purity and excellent adhesion of the metal film.


<img width="390" height="429" alt="image" src="https://github.com/user-attachments/assets/f7e37cfa-fd81-4e11-b816-ae4e40209fe5" />

###### 1.4 Lift-Off

The lift-off process was performed to remove the remaining photoresist, exposing only the desired gold conductive patterns. The substrates were immersed in an acetone and isopropanol (IPA) solution (90/10% by volume), assisted by an ultrasonic bath for 15 minutes. Final cleaning was done by rinsing with pure acetone followed by deionized water.

<img width="353" height="471" alt="image" src="https://github.com/user-attachments/assets/f88434c2-7d4a-46d9-b393-3db54d316d60" />

*Figure II.4 – Substrates immersed in acetone and IPA*

<img width="351" height="465" alt="image" src="https://github.com/user-attachments/assets/c747d07f-2c13-44b8-b788-b6b0e9e81af7" />

*Figure II.5 – Ultrasonic bath*

<img width="292" height="320" alt="image" src="https://github.com/user-attachments/assets/7675d133-ec40-464d-8a45-6da28e59a1fd" />

*Figure II.6 – Substrate rinsing*

###### 1.5 PEDOT:PSS Coating

To minimize impedance at the electrode–skin interface, a conductive organic layer of PEDOT:PSS (PH100 with DBSA, EG, and GOPS) was deposited by spin coating (1300 rpm, 50 s). A final bake at 150°C for 15 minutes ensured polymerization of the coating.

###### 1.6 Connection and Electrical Testing

Outside the cleanroom, thin conductive wires were attached to the electrodes using a two-part conductive epoxy adhesive (MG Chemicals 8331D), providing a robust electrical connection. The electrodes were then characterized by measuring their impedance using a WMP-3e impedance spectrometer over a frequency range from 10 Hz to 100 kHz, confirming their suitability for portable ECG applications.  

The results demonstrated a clear improvement in performance in terms of stability and impedance thanks to the use of the PEDOT:PSS layer, compared to electrodes coated only with gold.

<img width="440" height="341" alt="image" src="https://github.com/user-attachments/assets/5616b7af-a412-43b2-89c8-8438fc2b4e48" />

*Figure II.7– PEDOT electrode on two hands, 1Hz-100kHz, 50mV*

<img width="441" height="347" alt="image" src="https://github.com/user-attachments/assets/f45b1c94-cd7d-48d6-b01d-8ab326264f04" />


*Figure II.8– Gold only on two hands, 1Hz-100kHz, 50mV*

#### 2 Fabrication of the Passive OLED Matrix

###### 2.1 Deposition of Organic Layers

The deposition of organic layers was carried out in a high-vacuum thermal evaporation chamber (≈ 10⁻⁶ mbar) located in the cleanroom. This chamber is equipped with multiple independently controlled heating sources (tungsten crucibles), enabling sublimation of solid organic materials by Joule heating. Thanks to the ultra-clean environment and low pressure, the molecules pass directly from the solid phase to the vapor phase (sublimation) before condensing uniformly onto a substrate maintained at room temperature. This process ensures controlled, contamination-free growth of thin films.


The layer stack followed this order:
```
1. P++ (MoO₃, 0.5 nm): improves hole injection from the ITO anode.  
2. HTL (TCTA, 90 nm): transports holes to the emissive layer.  
3. EML (TPBi doped with 5% Ir(acac), 10 nm): exciton recombination zone and light emission.  
4. ETL (TPBi, 70 nm): transports electrons to the EML.  
5. HBL (LiF, 0.5 nm): blocks holes to optimize recombination.  
6. Cathode (Aluminum, 150 nm): injects electrons from the cathode.  
```
The deposition rate (typically between 0.1 and 1 Å/s) was regulated in real time using a quartz crystal microbalance, which precisely measures the deposited mass through the frequency shift of a piezoelectric crystal. This allows nanometer-level control of each layer’s thickness.

Stainless steel shadow masks, manually positioned between each deposition step, were used to define the pixel patterns (10 × 10 mm). For the EML layer, co-evaporation of the host material (TPBi) and the phosphorescent dopant (Ir(acac)) was implemented by adjusting the crucible temperatures (175°C and 285°C respectively) to achieve a 5 wt% doping ratio. This process ensures optimal molecular homogeneity, which is essential for maximizing recombination efficiency and luminance.  

A vacuum stabilization pause was maintained after each deposition step to stabilize the chamber and prevent cross-contamination between successive layers.

<img width="318" height="238" alt="image" src="https://github.com/user-attachments/assets/d98b7996-4825-4dd5-8501-541318896b1b" />

*Figure II.9 – Inside of the high-vacuum thermal evaporation chamber*

<img width="331" height="241" alt="image" src="https://github.com/user-attachments/assets/ca7e2aa8-8c72-4522-b630-246d5824578d" />

*Figure II.10 – Schematic of the final OLED matrix*

To control the quality of the depositions, the growth kinetics of each layer were monitored in real time. The following figure shows the simultaneous evolution of the deposition rate (in Å/s) and the vacuum level (in 10⁻⁶ mbar) as a function of time for the HTL layer. The corresponding curves for the remaining layers are provided in the appendix.  

After deposition of the final metallic layer (aluminum cathode), the organic stack is complete. The resulting OLED matrix is thus ready for encapsulation and characterization.

The figure below shows the matrix obtained at the end of the fabrication process.


<img width="948" height="514" alt="image" src="https://github.com/user-attachments/assets/2d209941-52b6-497c-91cf-c452819baf78" />

*Figure II.11 – Evolution of deposition rate and vacuum level as a function of process time (HTL layer)*

<img width="476" height="518" alt="image" src="https://github.com/user-attachments/assets/d1d32651-773c-4c1f-bd12-4cf1983067b4" />

*Figure II.12 – Final OLED matrix*


To validate the quality of the depositions, a series of characterizations was carried out on key layers of the device. The following section presents the thickness measurements obtained for ITO, aluminum, and the electron transport layer (ETL), using appropriate techniques.

###### 2.2 Thickness Characterizations: ITO, ETL, and Aluminum

The ITO, ETL (TPBi), and aluminum layers fulfill essential functions in the OLED architecture: ITO ensures hole injection from the transparent anode, ETL transports electrons to the recombination zone while blocking holes, and aluminum serves as the metallic cathode.  

Two complementary techniques were used for the characterization of deposited thicknesses: **mechanical profilometry** with the Bruker Dektak XTL system, and **spectroscopic ellipsometry** with the GES 5 setup.

<img width="533" height="407" alt="image" src="https://github.com/user-attachments/assets/609989b2-a178-4726-ba11-a81eccb03f82" />

*Figure II.13 – GES 5 Ellipsometer*

<img width="570" height="317" alt="image" src="https://github.com/user-attachments/assets/e3509cc0-c405-4f1f-95f8-3251499c2aa7" />

*Figure II.14 – Stylus profilometer*


The stylus profilometer relies on the vertical displacement of a stylus in contact with the surface, allowing measurement of step heights with angstrom-level resolution. The GES 5 ellipsometer, on the other hand, measures changes in the polarization state of light reflected by the sample; the ellipsometric parameters Ψ and ∆ are then used to deduce the thickness and refractive index of the layers through regression models.  

Although both approaches were explored, only the results obtained with the profilometer are presented in this report.

<img width="821" height="500" alt="image" src="https://github.com/user-attachments/assets/5959d099-3693-4f6b-925f-3c2581378083" />

Figure II.15 – 2D Profile of the OLED Matrix

The figure above shows a 2D thickness homogeneity map of the OLED matrix, obtained from multiple measurement lines taken with the stylus profilometer. Each horizontal line corresponds to a thickness profile measured along a lateral path, and the colors represent thickness variations in angstroms (Å). Homogeneous color regions indicate uniform deposition, while pronounced variations highlight non-uniformities. Very low values likely correspond to areas where the stylus could not properly track the surface — this may be due to poor contact, film irregularities, or abrupt transitions at the edges of the measured zone. These artifacts do not accurately reflect the actual material thickness and can be excluded from the analysis to retain only representative areas.

After evaluating the deposition homogeneity through thickness measurements, we now turn to the electrical and optical performance of the OLED matrix. The current density–voltage–luminance (J–V–L) characterization makes it possible to quantify its electroluminescent behavior and deduce key parameters such as external efficiency or threshold voltage.

###### 2.3 Current Density–Voltage–Luminance (J–V–L) Characterization

The performance of the OLED matrix was evaluated by measuring the current density J, the luminance L, and the external luminous efficiency ηlum as a function of the applied voltage V. These curves allow for the analysis of charge transport, the device’s optoelectronic behavior, and the identification of the optimal operating point.

<img width="826" height="583" alt="image" src="https://github.com/user-attachments/assets/b809c72d-4b8e-4402-87d3-b4e9002101e5" />

Figure II.16 – J–V–L curve in logarithmic scale

Analysis of the J–V curve in logarithmic scale reveals two distinct regimes:  
- At low voltage, the current follows ohmic behavior, characterized by a linear relationship between J and V, indicating conduction dominated by charge injection.  
- When V exceeds a certain critical threshold, the current becomes proportional to V², a regime described by the Mott–Gurney law (SCLC, Space-Charge Limited Current).  

This transition marks the shift to transport limited by carrier mobility in the organic layers.


By applying the Mott–Gurney law:

$$
J = \frac{9}{8} \, \varepsilon_r \varepsilon_0 \mu \, \frac{V^2}{d^3}
$$

with:  
- $\varepsilon_r = 3.6$ (relative permittivity of the organic layers),  
- $\varepsilon_0 = 8.85 \times 10^{-12} \, \text{F/m}$,  
- $d = 100 \, \text{nm} = 100 \times 10^{-9} \, \text{m}$,  
- $J = 2.8 \times 10^{-5} \, \text{A/cm}^2$,  
- $V = 5 \, \text{V}$.  

By substituting these values into the Mott–Gurney equation, the average charge carrier mobility is obtained as:

$$
\mu = 3.12 \times 10^{-13} \ \text{m}^2 \ / \ (\text{V·s})
$$

This very low value suggests that charge transport is strongly limited by the intrinsic carrier mobility in organic materials, which is a limiting factor for the overall OLED performance.  

The external luminous efficiency, denoted $\eta_{\text{lum}}$, is defined by the following relation:

$$
\eta_{\text{lum}} = \frac{L}{J} 
$$

Taking into account the stable operating region of the OLED, we experimentally determined an optimal external luminous efficiency $\eta_{\text{lum}}$ of **125 Cd/A**, obtained around **8 V**, with significant measured luminance and stable injection current. This value represents a good compromise between energy efficiency and light intensity, and was chosen as the reference operating point.  

Now that we have characterized the optoelectronic performance of the OLED matrix and ECG electrodes, we turn to the electronic and software aspects of the project. This section describes the architecture of the ECG signal conditioning circuit, as well as the programming of the microcontroller responsible for its acquisition, processing, and display.

## III Electronics and Programming

#### 1 Electronics and Programming (the "Machine")

The electronic circuit used for ECG signal acquisition is based on the **OCASS ECG board**, specifically developed for biopotential measurement. This board integrates an instrumentation amplifier coupled with analog filtering stages, allowing the extraction of the ECG signal from electrodes placed on the skin.

To power the OCASS ECG circuit, a DC voltage is applied between the test point **TP7** (positive) and the board’s **GND** (ground). An initial experimental verification was carried out by connecting the analog output of the OCASS circuit to an oscilloscope.

<img width="490" height="501" alt="image" src="https://github.com/user-attachments/assets/b5eadcc3-401a-45a2-894b-0276920865a2" />

Figure III.1 – The OCASS ECG board

<img width="584" height="436" alt="image" src="https://github.com/user-attachments/assets/3ee5a1e4-40b9-4fdd-9e69-50268920c7c3" />

Figure III.2 – First detection of the ECG signal

<img width="593" height="556" alt="image" src="https://github.com/user-attachments/assets/c40c3160-4de2-4c00-9b08-aa78848de33e" />

Figure III.3 – Shape of an ECG signal

The ECG signal was clearly detected, with a recognizable shape of the **P–QRS–T complex**, characteristic of human cardiac rhythm. However, this initial measurement also highlighted the presence of **parasitic noise**, particularly visible on the rising and falling edges of the signal. This noise can be attributed to several sources:

- subject movement or poor electrode–skin contact,  
- capacitive coupling with mains electricity,  
- ambient electromagnetic interference.  

Thus, although the OCASS circuit provides an exploitable signal, additional **digital processing** is necessary to improve signal stability and enable reliable downstream detection of the cardiac rhythm.


###### 1.1 Signal Filtering After Acquisition

The ECG signal from the OCASS board, although physiologically exploitable, remains strongly affected by noise due to various interferences: electromagnetic disturbances, subject movement, or power supply fluctuations. To obtain a digitally usable signal, preprocessing was implemented directly during acquisition.

The first processing stage relies on a **Finite Impulse Response (FIR) digital filter** designed with **50 coefficients**, adapted to a sampling frequency of **500 Hz**. This filter operates as a **band-pass filter** targeting frequencies between **1 Hz and 40 Hz**, which is the typical range of cardiac activity. It removes DC components (low-level noise, DC drift) and high frequencies (high-frequency noise, 50 Hz interference), while preserving the temporal characteristics of the ECG signal, such as the R-peaks.

Following the FIR filtering, a **Kalman filter** is applied to further smooth the signal. This adaptive filter, based on a probabilistic model, dynamically estimates the signal value by considering both measurement uncertainty (noise) and the expected evolution of the ECG signal. It is particularly effective in noisy environments, as it preserves fast transitions (R-peaks) while reducing unwanted fluctuations.

#### 2 Embedded Programming of the Microcontroller

The digital processing of the ECG signal is fully implemented on board the **STM32 microcontroller** using the **STM32CubeIDE** software. Peripheral configuration for ADC, Timers, and UART was performed with **CubeMX**.

The analog signal from the OCASS board is digitized via the **ADC1 analog-to-digital converter**, configured with external triggering by **Timer 6** to sample the signal at a fixed frequency (here, 500 Hz). The timer acts as an acquisition clock, automatically triggering an ADC conversion at each period.

The first processing applied is a **50th-order FIR band-pass filter**, defined by a table of constant coefficients adapted to the ECG signal’s bandwidth.  
The `fir()` function is implemented as follows:

```
1 float fir(float newValue) {
2 ...
3 output += h_coefficients[i] * adc_values [( indice + i) % (FilterOrder
+ 1)];
4 ...
5 }

```
Listing 1 – FIR Filter Implementation

To further smooth the signal while preserving the dynamics of the peaks, a **Kalman filter** is applied to the FIR output. The filter effectively reduces noise while estimating the true cardiac voltage:  
```
1 float kalman_adc = kalman_filter(volt);

```
Listing 2 – Call to the Kalman Filter

The filtered signal is then used to detect **R-peaks** (when the voltage exceeds a fixed threshold, here **2.0 V**). Each peak is interpreted as a heartbeat:  
```
1 if (amplifiedVoltage1 >= 2.0 && (currentTime - lastPeakTime > 200)) {
2 ledBlinkCount ++;
3 }

```
Listing 3 – R-Peak Detection

The number of detected R-peaks is converted into **BPM** every second, and a sliding average is computed over the last 5 values to smooth the display.
```
1 uint32_t count = ledBlinkCount * 60;
2 ajouter_lastcount(lastcount);
3 float moyenne_lastcount = calculer_moyenne_lastcount ();
4 MATRIX_DisplayMessage (0, bpm_str , strlen(bpm_str));
```
Listing 4 – BPM Calculation and Display on LED Matrix

Finally, the three signals (raw, FIR, Kalman) are sent via **UART** to a software tool such as **SerialPlot** for real-time graphical analysis:  
```
1 sprintf(MSG , "%.4f;%.4f;%.4f\r\n", voltage , volt , amplifiedVoltage1);
2 HAL_UART_Transmit (&huart1 , (uint8_t *)MSG , strlen(MSG), HAL_MAX_DELAY);

```
Listing 5 – UART Transmission to SerialPlot

<img width="1035" height="225" alt="image" src="https://github.com/user-attachments/assets/f7ee23e1-09c7-4d77-877c-770a76c0d35e" />

Figure III.4– Raw signal

<img width="1035" height="195" alt="image" src="https://github.com/user-attachments/assets/1826c9cf-d8b9-488f-bd0b-8659ebf8f61e" />

Figure III.5– FIR-filtered signal

<img width="1036" height="189" alt="image" src="https://github.com/user-attachments/assets/b251390f-f9ee-420b-9fc2-b271c6a22615" />

Figure III.6– Kalman-smoothed signal


###### 2.1 OLED Matrix (the "Interface")

The printed circuit board shown below drives an 8×8 LED matrix via an SPI interface. It is mainly based on the **MAX7219** integrated circuit, a LED matrix driver capable of sequentially controlling up to 64 LEDs.  
This circuit enables the dynamic display of data, such as the measured heart rate, scrolling from right to left across the LED matrix.

<img width="583" height="317" alt="image" src="https://github.com/user-attachments/assets/98adff76-bf0f-4cd7-8c63-4692c433d757" />

Figure III.7– PCB of the OLED matrix control board

<img width="448" height="377" alt="image" src="https://github.com/user-attachments/assets/4aebf1a3-df4b-4185-9115-ba1e9bd07a3d" />

Figure III.8– Shape of an ECG signal

The BPM display is performed using the following function in the embedded code:  
```
MATRIX_DisplayMessage(0, bpm_str, strlen(bpm_str));
```
This instruction sends the string containing the value of the average BPM (calculated over the last 5 measured values) to the LED matrix. The message is updated every second, allowing real-time reading of cardiac activity on the display.

---

## IV Conclusion

This multidisciplinary project enabled us to design and implement a complete chain for detecting, processing, and displaying a physiological signal—specifically, the heart rate. It combined multiple fields, ranging from materials physics to embedded electronics, including cleanroom fabrication technologies.  

From a technological standpoint, we fabricated flexible electrodes on a polyimide substrate, suitable for portable use and compliant with biomedical constraints. The fabrication of the OLED matrix, the core of the display device, required a precise sequence of organic layer depositions through high-vacuum thermal evaporation. The quality of each layer (HTL, ETL, Al, ITO, etc.) was validated through metrology techniques such as stylus profilometry and ellipsometry, ensuring the uniformity and optoelectronic performance of the device.  

From an electronics perspective, we designed and used an analog circuit to acquire the ECG signal, amplifying and conditioning it prior to digitization. The signal was then digitally processed by an STM32 microcontroller using a chain of digital filters (FIR, Kalman), followed by analysis to detect R-peaks and calculate heart rate. Finally, the results were dynamically displayed on an LED matrix driven by SPI, with smoothing ensured by embedded algorithms.  

This project allowed us to leverage cross-disciplinary skills in microfabrication, biomedical instrumentation, signal processing, and embedded programming, while addressing a concrete engineering challenge.  

We warmly thank our supervisors for their availability, technical guidance, and support throughout this Project Unit. Their expertise was invaluable in overcoming the challenges encountered and deepening our understanding of this interface between applied physics, electronics, and life sciences.

## V Annexes

#### A. Picture of the Complete System

<img width="582" height="740" alt="image" src="https://github.com/user-attachments/assets/7f0052de-e5ce-43fe-9cb1-4359bd658d54" />

Figure V.1 – ECG Device

---

#### B. Complementary Curves (OLED Matrix)

<img width="981" height="545" alt="image" src="https://github.com/user-attachments/assets/174b28c4-ac73-496d-81da-a8d8b9374d01" />

Figure V.2 – Evolution of deposition rate and vacuum level as a function of process time (HBL layer)

<img width="976" height="523" alt="image" src="https://github.com/user-attachments/assets/aa254ea5-262b-46c4-a49e-236a515ce53f" />

Figure V.3 – Evolution of deposition rate and vacuum level as a function of process time (MoO₃ layer)

<img width="973" height="530" alt="image" src="https://github.com/user-attachments/assets/67d8128c-dc6a-4e31-8da9-0971768dd4e5" />

Figure V.4 – Evolution of deposition rate and vacuum level as a function of process time (ETL layer)

<img width="983" height="543" alt="image" src="https://github.com/user-attachments/assets/d7ce3cf0-a057-4ddb-8cf5-b86728bf4aa6" />

Figure V.5 – Evolution of deposition rate and vacuum level as a function of process time (Cathode)



