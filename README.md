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


```
La figure ci-dessous présente la matrice réalisée à l’issue du procédé de fabrication.
```

```
Figure II.11– Evolution de la vitesse du dépôt et le niveau du vide en fonction du
temps de process (Couche HTL)
```
```
Figure II.12– Matrice OLED finale
```
Pour valider la qualité des dépôts réalisés, une série de caractérisations a été effectuée
sur certaines couches clés du dispositif. La section suivante présente les mesures d’épais-
seur obtenues pour l’ITO, l’aluminium, ainsi que pour la couche de transport d’électrons
(ETL), à l’aide de techniques adaptées.

###### 2.2 Caractérisations des épaisseurs : ITO, ETL et aluminium

Les couches d’ITO, d’ETL (TPBi) et d’aluminium remplissent des fonctions essen-
tielles dans l’architecture OLED : l’ITO assure l’injection des trous depuis l’anode trans-
parente, l’ETL transporte les électrons jusqu’à la zone de recombinaison tout en bloquant
les trous, et l’aluminium joue le rôle de cathode métallique.

Deux techniques complémentaires ont été utilisées pour la caractérisation des épais-
seurs déposées : laprofilométrie mécaniqueavec le systèmeBruker Dektak XTL, et


l’ellipsométrie spectroscopiqueavec le dispositifGES 5.

```
Figure II.13– L’ellipsomètre GES 5 Figure II.14– Profileur à stylet
```
Le profileur à stylet repose sur le déplacement vertical d’un stylet en contact avec la
surface, permettant de mesurer des hauteurs d’étape avec une résolution angströmique.
L’ellipsomètre GES 5, quant à lui, mesure les modifications d’état de polarisation d’une lu-
mière réfléchie par l’échantillon ; les paramètres ellipsométriquesΨet∆sont utilisés pour
en déduire l’épaisseur et l’indice optique des couches via des modèles de régression. Bien
que les deux approches aient été explorées, seuls les résultats obtenus avec le profilomètre
sont présentés dans ce rapport.

```
Figure II.15– Profile 2D de la matrice OLED
```
La figure ci-dessus représente une carte 2D d’homogénéité d’épaisseur de la matrice
OLED, obtenue à partir de plusieurs lignes de mesure réalisées avec le profileur à stylet.
Chaque ligne horizontale correspond à un profil d’épaisseur mesuré sur une trajectoire
latérale, et les couleurs traduisent les variations d’épaisseur en angströms (Å). Les zones
de couleur homogène témoignent d’un dépôt régulier, tandis que les variations marquées
indiquent des non-uniformités. Les valeurs très basses correspondent probablement à des


zones où le stylet n’a pas pu suivre correctement la surface — cela peut être dû à un
mauvais contact, à des irrégularités du film ou à des transitions abruptes en bordure de
zone mesurée. Ces artefacts ne reflètent pas fidèlement l’épaisseur réelle du matériau et
peuvent être exclues de l’analyse pour ne conserver que les zones représentatives.

Après avoir évalué l’homogénéité du dépôt par des mesures d’épaisseur, nous nous
intéressons à présent aux performances électriques et optiques de la matrice OLED. La
caractérisation courant-densité, tension et luminance (J–V–L) permet de quantifier son
comportement électroluminescent et d’en déduire des paramètres clés tels que le rende-
ment externe ou la tension de seuil.

###### 2.3 Caractérisation courant-densité, tension et luminance (J–V–L)

Les performances de la matrice OLED ont été évaluées par la mesure de la densité
de courantJ, de la luminanceLet du rendement lumineux externe ηlum en fonction
de la tension appliquéeV. Ces courbes permettent d’analyser le transport de charge, le
comportement optoélectronique du dispositif, et d’identifier le point de fonctionnement
optimal.

```
Figure II.16– Courbe J–V-L en échelle logarithmique
```
L’analyse de la courbeJpVqen échelle logarithmique révèle deux régimes distincts :
à faible tension, le courant suit un comportement ohmique, caractérisé par une relation
linéaire entreJetV, indiquant une conduction dominée par l’injection de charges. Lorsque
V dépasse une certaine tension critique, le courant devient proportionnel àV^2 , régime
décrit par la loi de Mott-Gurney (SCLC, Space-Charge Limited Current). Cette transition
marque le passage à un transport limité par la mobilité des porteurs dans les couches
organiques.


```
En appliquant la loi de Mott-Gurney :
```
```
J“
```
```
9
8
```
```
εrε 0 μ
```
```
V^2
d^3
```
avec :
— εr“ 3. 6 (permittivité relative des couches organiques),
— ε 0 “ 8. 85 ˆ 10 ́^12 F{m,
— d“100 nm“ 100 ˆ 10 ́^9 m,
— J“ 2 , 8 ˆ 10 ́^5 A{cm^2 ,
— V “5 V.
En injectant ces valeurs dans l’équation de Mott-Gurney, on obtient une mobilité
moyenne des porteurs de charge :

```
μ“ 3 , 12 ˆ 10 ́^13 m^2 {V ̈s
```
Cette valeur très faible suggère que le transport de charges est fortement limité par
la mobilité intrinsèque des porteurs dans les matériaux organiques, ce qui constitue un
facteur limitant pour les performances globales de l’OLED.
Le rendement lumineux externe, notéηlum, est défini par la relation suivante :

```
ηlum“
```
```
L
J
```
```
(1)
```
En tenant compte de la zone stable de fonctionnement de l’OLED, nous avons dé-
terminé expérimentalement un rendement lumineux externe optimalηlumde125 Cd/A,
obtenu autour de8 V, pour une luminance mesurée significative et un courant d’injec-
tion stable. Cette valeur correspond à un bon compromis entre efficacité énergétique et
intensité lumineuse, et a été retenue commepoint de fonctionnement de référence.

Maintenant que nous avons caractérisé les performances optoélectroniques de la ma-
trice OLED et des électrodes ECG, nous nous intéressons désormais à la partie électro-
nique et logicielle du projet. Cette section décrit l’architecture du circuit de conditionne-
ment du signal ECG, ainsi que la programmation du microcontrôleur en charge de son
acquisition, traitement et affichage.

## III Électronique et Programmation

#### 1 Electronique et Programmation (la "Machine")

Le circuit électronique utilisé pour l’acquisition du signal ECG repose sur la carte
OCASS ECG, développée spécifiquement pour la mesure biopotentielle. Cette carte in-
tègre un amplificateur instrumentation couplé à des étages de filtrage analogiques per-
mettant d’extraire le signal ECG à partir d’électrodes posées sur la peau.

Pour alimenter le circuit OCASS ECG, une tension continue est appliquée entre le
point de testTP7(positif) et leGND (masse) de la carte. Une première vérification


```
Figure III.1– La carte OCASS ECG
```
expérimentale a été réalisée en connectant la sortie analogique du circuit OCASS à un
oscilloscope.

```
Figure III.2– Première détection du
signal ECG Figure III.3– Forme d’un signal ECG
```
Le signal ECG a pu être clairement détecté, avec une forme reconnaissable du complexe
P-QRS-T, caractéristique du rythme cardiaque humain. Cependant, cette mesure initiale
met également en évidence la présence debruit parasite, visible notamment sur les
flancs montants et descendants du signal. Ce bruit peut être attribué à plusieurs sources :

```
— mouvements du sujet ou mauvaise qualité du contact des électrodes,
— couplage capacitif avec le secteur,
— interférences électromagnétiques ambiantes.
```
Ainsi, bien que le circuit OCASS fournisse un signal exploitable, un traitement
numérique complémentaire est nécessaire pour améliorer la stabilité du signal et permettre
une détection fiable du rythme cardiaque en aval.


###### 1.1 Filtrage des signaux après acquisition

```
Le signal ECG issu de la carte OCASS, bien que physiologiquement exploitable, reste
fortement bruité en raison de différents parasites : interférences électromagnétiques, mou-
vements du sujet, ou fluctuations de l’alimentation. Pour obtenir un signal utilisable nu-
mériquement, un prétraitement numérique a été mis en œuvre dès la phase d’acquisition.
```
```
Le premier étage de traitement repose sur unfiltre numérique de type FIR(Finite
Impulse Response) conçu sur50 coefficients, adaptés à une fréquence d’échantillonnage
de500 Hz. Ce filtre agit comme unpasse-bandeciblant les fréquences comprises entre
1 Hzet40 Hz, plage typique de l’activité cardiaque. Il permet de supprimer les compo-
santes continues (bruit de bas niveau, dérive DC) ainsi que les hautes fréquences (bruit
haute fréquence, interférences 50 Hz), tout en conservant les caractéristiques temporelles
du signal ECG comme les pics R.
```
```
À la suite du filtrage FIR, unfiltre de Kalmanest appliqué pour optimiser le lissage
du signal. Ce filtre adaptatif, fondé sur un modèle probabiliste, estime dynamiquement
la valeur du signal en tenant compte de l’incertitude de mesure (bruit) et de l’évolution
attendue du signal ECG. Il s’agit d’une solution particulièrement efficace dans les envi-
ronnements bruités, car elle conserve les transitions rapides (pics R) tout en réduisant les
fluctuations indésirables.
```
#### 2 Programmation embarquée du microcontrôleur

```
Le traitement numérique du signal ECG est entièrement réalisé à bord du microcon-
trôleur STM32 via le logiciel STM32CubeIDE. La configuration des périphériques ADC,
Timers et UART a été effectuée à l’aide du CubeMX.
```
```
Le signal analogique issu de la carte OCASS est numérisé via le convertisseur analogique-
numériqueADC1, configuré en déclenchement externe parTimer 6pour échantillonner
le signal à une fréquence fixe (ici 500 Hz). Le timer agit comme une horloge d’acquisition,
déclenchant automatiquement une conversion ADC à chaque période.
```
```
Le premier traitement appliqué est unfiltre passe-bande FIRd’ordre 50, défini par
un tableau de coefficients constants adapté à la bande passante du signal ECG.
La fonctionfir()est implémentée comme suit :
```
1 float fir(float newValue) {
2 ...
3 output += h_coefficients[i] * adc_values [( indice + i) % (FilterOrder
+ 1)];
4 ...
5 }

```
Listing 1 – Implémentation du filtre FIR
```
```
Pour lisser davantage le signal tout en conservant la dynamique des pics, un filtre
de Kalman est appliqué sur la sortie FIR. Le filtre réduit efficacement le bruit tout en
estimant la tension réelle du cœur :
```
1 float kalman_adc = kalman_filter(volt);

```
Listing 2 – Appel au filtre de Kalman
```

```
Le signal filtré est ensuite utilisé pour détecter les pics R (lorsque la tension dépasse
un seuil fixé, ici 2.0 V). Chaque pic est interprété comme un battement cardiaque :
```
1 if (amplifiedVoltage1 >= 2.0 && (currentTime - lastPeakTime > 200)) {
2 ledBlinkCount ++;
3 }

```
Listing 3 – Détection de pics R
```
```
Le nombre de pics R détectés est converti en BPM toutes les secondes, et une moyenne
glissante est calculée sur les 5 dernières valeurs pour lisser l’affichage :
```
1 uint32_t count = ledBlinkCount * 60;
2 ajouter_lastcount(lastcount);
3 float moyenne_lastcount = calculer_moyenne_lastcount ();
4 MATRIX_DisplayMessage (0, bpm_str , strlen(bpm_str));

```
Listing 4 – Calcul du BPM et affichage sur matrice LED
```
```
Enfin, les trois signaux (brut, FIR, Kalman) sont envoyés via l’UART vers un logiciel
commeSerialPlotpour analyse graphique en temps réel :
```
1 sprintf(MSG , "%.4f;%.4f;%.4f\r\n", voltage , volt , amplifiedVoltage1);
2 HAL_UART_Transmit (&huart1 , (uint8_t *)MSG , strlen(MSG), HAL_MAX_DELAY);

```
Listing 5 – Transmission UART vers SerialPlot
```
```
Figure III.4– Signal brut
```
```
Figure III.5– Signal filtré FIR
```
```
Figure III.6– Signal lissé Kalman
```

###### 2.1 Matrice OLEDs (l "Interface")

Le circuit imprimé présenté ci-dessous permet de piloter une matrice LED 8x8 via
une interface SPI. Il repose principalement sur le circuit intégréMAX7219, un pilote de
matrice LED capable de contrôler jusqu’à 64 LED de manière séquentielle.
Le circuit permet ainsi l’affichage dynamique de données, telles que le rythme cardiaque
mesuré, en défilant de droite à gauche sur la matrice LED.

```
Figure III.7– Circuit imprimé de la
carte de commande de la matrice OLED Figure III.8– Forme d’un signal ECG
```
```
L’affichage du BPM s’effectue grâce à la fonction suivante dans le code embarqué :
```
MATRIX_DisplayMessage(0, bpm_str, strlen(bpm_str));

Cette instruction envoie la chaîne contenant la valeur du BPM moyen (calculéè sur
les 5 dernières valeurs mesurées) à la matrice LED. Le message est actualisé toutes les
secondes, ce qui permet une lecture en temps réel de l’activité cardiaque sur l’afficheur.

## IV Conclusion

Ce projet pluridisciplinaire nous a permis de concevoir et de mettre en œuvre une
chaîne complète de détection, de traitement et d’affichage d’un signal physiologique, en
l’occurrence le rythme cardiaque. Il a combiné plusieurs domaines allant de la physique
des matériaux à l’électronique embarquée, en passant par les technologies de fabrication
en salle blanche.
Sur le plan technologique, nous avons réalisé des électrodes souples sur un substrat en
polyimide, adaptées à un usage portable et conformes aux contraintes biomédicales. La fa-
brication de la matrice OLED, cœur du dispositif d’affichage, a nécessité un enchaînement
précis de dépôts de couches organiques par évaporation thermique sous vide poussé. La
qualité de chaque couche (HTL, ETL, Al, ITO, etc.) a été validée par des caractérisations
métrologiques telles que la profilométrie à stylet et l’ellipsométrie, permettant d’assurer
l’homogénéité et la performance optoélectronique du dispositif.
D’un point de vue électronique, nous avons conçu et utilisé un circuit analogique de
récupération du signal ECG, en l’amplifiant et le conditionnant avant numérisation. Ce
signal a ensuite été traité numériquement par un microcontrôleur STM32 via une suite de
filtres numériques (FIR, Kalman), puis analysé afin de détecter les pics R et calculer la


fréquence cardiaque. Enfin, les résultats ont été affichés dynamiquement sur une matrice
LED pilotée par SPI, et lissés grâce à des algorithmes embarqués.
Ce projet nous a permis de mobiliser des compétences transverses en microfabrication,
instrumentation biomédicale, traitement du signal et programmation embarquée, tout en
répondant à un enjeu d’ingénierie concret.
Nous tenons à remercier chaleureusement nos encadrants pour leur disponibilité, leurs
conseils techniques et leur accompagnement tout au long de cette Unité Projet. Leur
expertise a été précieuse pour surmonter les défis rencontrés et approfondir notre compré-
hension de cette interface entre physique appliquée, électronique et sciences du vivant.

## V Annexes

#### A. Photo du système complet

```
Figure V.1– Dispositif ECG
```

#### B. Courbes complémentaires(Matrice OLED)

```
Figure V.2– Evolution de la vitesse du dépôt et le niveau du vide en fonction du
temps de process (Couche HBL)
```
```
Figure V.3– Evolution de la vitesse du dépôt et le niveau du vide en fonction du
temps de process (Couche MoO3)
```

Figure V.4– Evolution de la vitesse du dépôt et le niveau du vide en fonction du
temps de process (Couche ETL)

Figure V.5– Evolution de la vitesse du dépôt et le niveau du vide en fonction du
temps de process (Cathode)


## Références


