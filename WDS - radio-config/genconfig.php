<?php
// works with version 3.2.11.0 of the Silicon Labs WDS software


exec("ls -1 *.h | grep .h | grep -v 'rfconfig'", $files);

$count = 0;
$rxlist = array();
$txlist = array();
foreach ($files as $filename) {
	//	echo $filename."\n";
	$file = file_get_contents($filename);
	$filelines = explode("\n", $file);

	$started = false;
	
// DECODE CONFIG FILE TYPE
	list($proto, $mode, $freq, $bps, $level) = explode("_", $filename);
	list($level, $bla) = explode(".", $level, 2);
	$origmode = $mode;
	//echo "bps=$bps, proto=$proto, mode=$mode, level=$level; \n";
	switch ($proto) {
		case "FLEX":
			switch ($mode) {
				case "RX":
					$rxconfig = true;
					if ($bps == "1600") $mode = 1;
					if (($bps == "3200") && ($level == "2")) $mode = 2;
					if (($bps == "3200") && ($level == "4")) $mode = 3;
					if (($bps == "6400") && ($level == "4")) $mode = 4;
					break;
				case "TX":
					$mode = 5;
					$rxconfig = false;
					break;
			}
			break;
		case "POCSAG":
			switch ($mode) {
				case "RX":
					$rxconfig = true;
					if ($bps == "512") $mode = 6;
					if ($bps == "1200") $mode = 7;
					if ($bps == "2400") $mode = 8;
					break;
				case "TX":
					$mode = 9;
					$rxconfig = false;
					break;
			}
			break;
                case "ERMES":
                        $rxconfig = false;
			$mode = 0;
                        break;

	}
// END DECODE CONFIG FILE TYPE

// MAKE LIST WITH RELEVANT PROPERTIES
	if(($mode==2)||($mode==3)||($mode==4)){
		$shortenedconfig = true;
		$parent = $proto."_".$origmode."_".$freq."_1600_2.h";
		exec("diff $parent $filename | grep '> #define'", $shortlist);
		$properties = array();
		$shortproperties = array();
		foreach($shortlist as $line){
			$line = substr($line,10);
			list($property,) = explode(" ",$line,2);
			$proplist = array();
			$proplist = explode(",", $line);
			$len = count($proplist);
			$len = dechex($len);
			if(strlen($len)==1)$len = "0".$len;
			$len = "0x".$len;
			$shortproperties[$property] = $len;
		} 
		//print_r($shortproperties);
	} else {
		$shortenedconfig = false;
	}
	
	$properties = array();
	foreach ($filelines as $line) {
		$line = trim($line);
		if (stripos($line, "// RF Freq.(MHz)") !== false) {
			$arr = explode(" ", $line);
			$freq = $arr[3];
			//echo $freq."\n";
		}
		if (stripos($line, "#define RADIO_CONFIGURATION_DATA_ARRAY { \\") !== false) {
			$started = true;
		} else if ($started) {
			$line = trim(trim(trim($line, "\\")), ",");
			if (substr($line, 0, 2) == "0x" && substr($line, 0, 4) != "0x00") {
				list($len, $property) = explode(",", $line);
				$len = trim($len);
				$property = trim($property);
				if($shortenedconfig){
						$cut = array();
						$cut['RF_POWER_UP'] = 1; // not needed!
						$cut['RF_GPIO_PIN_CFG']=1; // not needed!
						$cut['RF_GLOBAL_XO_TUNE_2']=1; // not needed!
						$cut['RF_GLOBAL_CONFIG_1']=1; // not needed!
						
								//$cut['RF_INT_CTL_ENABLE_4']=1; // don't cut these
								//$cut['RF_FRR_CTL_A_MODE_4']=1; // don't cut these
								
								//$cut['RF_PKT_FIELD_3_CRC_CONFIG_12']=1; // not needed
						$cut['RF_PKT_CRC_SEED_31_24_4']=1; // not needed
 						//$cut['RF_PA_TC_1']=1; // not needed
								//$cut['RF_PKT_RX_FIELD_1_CRC_CONFIG_12']=1; // not needed
								//$cut['RF_PKT_RX_FIELD_4_CRC_CONFIG_5']=1; // not needed
						//$cut['RF_SYNTH_PFDCP_CPFF_7']=1; // not needed
						//$cut['RF_MATCH_VALUE_1_12']=1; // not needed
						//$cut['RF_FREQ_CONTROL_INTE_8']=1; // don't cut this!
						
						//if(isset($cut[$property])){
						//} else {
							$properties[$property] = $len;
						//}
				} else {
					$properties[$property] = $len;
				}
			}
		}
		if ($started && substr($line, 0, 1) == "}") {
			$started = false;
		}
	}

	//print_r($properties);
// END MAKE LIST OF RELEVANT PROPERTIES
	
	
	$config = chr(count($properties));

	foreach ($properties as $property => $len) {
		foreach ($filelines as $line) {
			$line = trim($line);
			$search = "#define $property ";
			if (substr($line, 0, strlen($search)) == $search) {
				$arr = explode(" ", $line, 3);
				$bytes = $arr[2];
				$bytes = $len . ", " . $bytes;
				$bytes = str_replace("0x", "", $bytes);
				$bytes = str_replace(", ", "", $bytes);
				$config .= hex2bin($bytes);
			}
		}
	}
	


	$field = bin2hex($config);
	$field = chunk_split($field, 2, ", 0x");
	$field = ", 0x" . substr($field, 0, -2);
	$hex = substr($field, 2, -2);
	if ($rxconfig) {
		echo "static const uint8_t PROGMEM rf4463RXsetup$count" . "[] = {" . $hex . "}; // $freq RX\n";
		$rxlist[($freq * 1000000) + $mode] = $count;
	} else {
		echo "static const uint8_t PROGMEM rf4463TXsetup$count" . "[] = {" . $hex . "}; // $freq TX\n";
		$txlist[($freq * 1000000) + $mode] = $count;
	}
	$count++;
}

$farr = "static const uint32_t PROGMEM rxFrequencies[] = {";
foreach ($rxlist as $entry => $c) {
	$farr .= $entry . ", ";
}
$farr = substr($farr, 0, -2);
$farr .= "};\n";
echo $farr;

$farr = "static const uint32_t PROGMEM txFrequencies[] = {";
foreach ($txlist as $entry => $c) {
	$farr .= $entry . ", ";
}
$farr = substr($farr, 0, -2);
$farr .= "};\n";
echo $farr;


$carr = " __attribute__ ((unused)) static const uint8_t* PROGMEM configs[] = {";
foreach ($rxlist as $entry => $c) {
	$carr .= "rf4463RXsetup" . $c . ", ";
}
foreach ($txlist as $entry => $c) {
	$carr .= "rf4463TXsetup" . $c . ", ";
}
$carr = substr($carr, 0, -2);
$carr .= "};\n";
echo $carr;
echo "#define RX_FREQS " . count($rxlist) . "\n";
echo "#define TX_FREQS " . count($txlist) . "\n";

