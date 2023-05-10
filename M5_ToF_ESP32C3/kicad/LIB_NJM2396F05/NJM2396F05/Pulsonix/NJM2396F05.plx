PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//364257/1101914/2.50/4/4/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c174_h116"
		(holeDiam 1.16)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 1.74) (shapeHeight 1.74))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 1.74) (shapeHeight 1.74))
	)
	(padStyleDef "s174_h116"
		(holeDiam 1.16)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.74) (shapeHeight 1.74))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 1.74) (shapeHeight 1.74))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "TO254P480X1000X1900-4P" (originalName "TO254P480X1000X1900-4P")
		(multiLayer
			(pad (padNum 1) (padStyleRef s174_h116) (pt 0, 0) (rotation 90))
			(pad (padNum 2) (padStyleRef c174_h116) (pt 2.54, 0) (rotation 90))
			(pad (padNum 3) (padStyleRef c174_h116) (pt 5.08, 0) (rotation 90))
			(pad (padNum 4) (padStyleRef c174_h116) (pt 7.62, 0) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -1.59 3.4) (pt 9.21 3.4) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 9.21 3.4) (pt 9.21 -1.9) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 9.21 -1.9) (pt -1.59 -1.9) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -1.59 -1.9) (pt -1.59 3.4) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.34 3.15) (pt 8.96 3.15) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 8.96 3.15) (pt 8.96 -1.65) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 8.96 -1.65) (pt -1.34 -1.65) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.34 -1.65) (pt -1.34 3.15) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.34 1.88) (pt -0.07 3.15) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt 8.96 -1.65) (pt 8.96 3.15) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 8.96 3.15) (pt -1.34 3.15) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.34 3.15) (pt -1.34 0) (width 0.2))
		)
	)
	(symbolDef "NJM2396F05" (originalName "NJM2396F05")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 1200 mils 100 mils) (width 6 mils))
		(line (pt 1200 mils 100 mils) (pt 1200 mils -400 mils) (width 6 mils))
		(line (pt 1200 mils -400 mils) (pt 200 mils -400 mils) (width 6 mils))
		(line (pt 200 mils -400 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 1250 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 1250 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "NJM2396F05" (originalName "NJM2396F05") (compHeader (numPins 4) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "VIN") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "VOUT") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "GND") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "ON/OFF CONTROL") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "NJM2396F05"))
		(attachedPattern (patternNum 1) (patternName "TO254P480X1000X1900-4P")
			(numPads 4)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
			)
		)
		(attr "Mouser Part Number" "513-NJM2396F05")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/NJR/NJM2396F05?qs=uTtZEXA6qg9fe3K1fQPnMA%3D%3D")
		(attr "Manufacturer_Name" "New Japan Radio")
		(attr "Manufacturer_Part_Number" "NJM2396F05")
		(attr "Description" "LDO Voltage Regulators LDO w/On/Off Cntrl")
		(attr "<Hyperlink>" "https://datasheet.datasheetarchive.com/originals/distributors/Datasheets-DGA25/1802734.pdf")
		(attr "<Component Height>" "4.8")
		(attr "<STEP Filename>" "NJM2396F05.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)
