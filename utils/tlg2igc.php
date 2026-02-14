<?
ini_set("display_errors", 1);
header('Content-type: text/plain');
require_once('ogn_tlg.inc');
date_default_timezone_set('UTC');

$prog_name = 'tlg2igc.php';			// this appears in IGC files HOSOF header
$version = '1.0-RC1';				// this appears in IGC files HOSOF header

define("TLG_DIR",'LOGS/raw/');			// ending slash is mandatory!
define("IGC_DIR",'LOGS/');			// ending slash is mandatory!
define("TRACKERS_DICTIONARY",'trackers.csv');	// translate tracker IDs to own text (glider reg, CID, pilot name etc.)
						// example line: 071A2B3C,myGlider


//
// Receive TLG file
//

if ($_SERVER['REQUEST_METHOD'] == 'POST') {

  // is received filename safe?
  if (!ctype_xdigit($_SERVER['HTTP_X_FILE_NAME'][0]) || strpos($_SERVER['HTTP_X_FILE_NAME'],'/') !== false)
    $_SERVER['HTTP_X_FILE_NAME'] = 'unknown.TLG';

  $trackerSubDir = trackerSubDir($_SERVER['HTTP_X_FILE_NAME'] ?? NULL, $_SERVER['HTTP_X_OGNTRACKER_ACFTID'] ?? NULL);
  if (!is_dir(TLG_DIR.$trackerSubDir)) mkdir(TLG_DIR.$trackerSubDir, 0777, true);
  if (!is_dir(IGC_DIR.$trackerSubDir)) mkdir(IGC_DIR.$trackerSubDir, 0777, true);

  $TLGfilename = TLG_DIR.$trackerSubDir.substr($_SERVER['HTTP_X_FILE_NAME'] ?? 'unknown.TLG',0,-4).'_'.uniqid().'.TLG';

  $fIN = fopen('php://input', 'r');
  $fTLG = @fopen($TLGfilename, 'w');

  if ($fTLG === false) {
    http_response_code(500);
    echo "ERROR writing $TLGfilename\n";
    die();
  }

  $bytes = 0;
  while (!feof($fIN)) {
    $bytes += fwrite($fTLG,fread($fIN,1024));

    if ($bytes > 102400) {				// 100kB
      fclose($fIN);
      fclose($fTLG);
      http_response_code(500);
      die("input file too big");
    };
  };

  fclose($fIN);
  fclose($fTLG);

} else {
  http_response_code(403);
  die('Forbidden');
};



//
// Parse TLG and convert to IGC
//

$d = parse_tlg($TLGfilename);

if ($d['file_mac'] === NULL)
  $d['file_mac'] = $_SERVER['HTTP_X_OGNTRACKER_MAC'] ?? '';
if ($d['file_id'] === NULL)
  $d['file_id'] = $_SERVER['HTTP_X_OGNTRACKER_ACFTID'] ?? NULL;

if ($d['file_id'] === NULL) {
  http_response_code(500);
  echo "ERROR reading device ID\n";
  die();
};


$d['info'] = first_pass($d);

write_igc($d,IGC_DIR.$trackerSubDir);

http_response_code(200);
header('Content-Length: 10');
die("Upload OK.");



function first_pass($data) {

  // scan fields for optional info
  $info = array(			// pre-fill IGC mandatory fields
    'tracker_id' => (int) hexdec($data['file_id']) & 0xffffff,
    'first_pos_time' => NULL,
    'last_pos_time' => NULL,
    'Pilot' => '',
    'PilotID' => '',
    'Crew' => '',
    'Type' => '',
    'ID' => '',
    'Reg' => '',
    'Class' => '',
    'firmware' => '',
    'hardware' => '',
    'have' => array (
      	'pressure' => false,
      	'temp' => false,
      	'hum' => false,
        'voltage' => false,
	      'oxygen' => false,
      	'pulse' => false,
	      'audio_noise' => false,
	      'turnrate' => false,
      	'climbrate' => false,
    ),
  );


  foreach ($data['entries'] ?? array() as $e_id => $e) {
    if (($e['ogn_rx'] ?? NULL) == 1)				// foreign (radio-received) packet
      continue;
    if ($e['h_address'] != $info['tracker_id'])			// not our packet - that shouldn't happen!
      continue;
    if (!isset($e['report_type']))
      continue;

    if ($e['report_type'] === 1) {			// info packet
      foreach ($e['info'] as $_k => $_v)
        $info[$_k] = $_v;
      continue;
    };


    if ($e['report_type'] === -1) {			// position packet
      if ($e['turnrate'] ?? NULL > 0) $info['have']['turnrate'] = true;
      if ($e['climbrate'] ?? NULL > 0) $info['have']['climbrate'] = true;

      $info['last_pos_time'] = $e['unixtime'];
      if ($info['first_pos_time'] === NULL)
        $info['first_pos_time'] = $e['unixtime'];
    };

    if ($e['report_type'] === 0) {			// status packet
      if ($e['sts_pressure'] ?? NULL !== NULL) $have['pressure'] = true;
      if ($e['sts_temp'] ?? NULL !== NULL) $info['have']['temp'] = true;
      if ($e['sts_hum'] ?? NULL !== NULL) $info['have']['hum'] = true;
      if ($e['voltage'] ?? NULL !== NULL) $info['have']['voltage'] = true;
      # if ($e['audio_noise'] ?? NULL > 0) $info['have']['audio_noise'] = true;		// listed in specs, but not used
      # if ($e['oxygen'] ?? NULL > 0) $info['have']['oxygen'] = true;			// listed in specs, but not used
      # if ($e['pulse'] ?? NULL > 0) $info['have']['pulse'] = true;			// listed in specs, but not used

      if (($e['firmware'] ?? NULL) !== NULL) {
        $info['firmware'] = $e['firmware'];
      }
      if (($e['hardware'] ?? NULL) !== NULL)
        $info['hardware'] = $e['hardware'];
    };
  };

  return ($info);
};


function write_igc($data, $dir = './') {
  define("IGCEOL","\r\n");

  if (substr($dir,-1) != '/')
    $dir .= '/';

  $info = $data['info'];

  $file_serial = 0;


  // check if this is a new flight or continuation of previous file
  // this assumes tracker always uploads oldest file first

  $append_igc_file = false;

  while (true) {
    $file_serial++;
    $IGCfilename = date('Y-m-d',$data['file_time']) . '-XOG-' . substr($data['file_id'],-6) .
		sprintf("-%02d.IGC",$file_serial);

    if (file_exists($dir.$IGCfilename)) {
      $_append = check_append($dir.$IGCfilename,$data);
      if ($_append === -1) {				// same TLG file, overwrite old IGC
        unlink($dir.$IGCfilename);
        break;
      };

      if ($_append === true) {
        $append_igc_file = true;			// flight continues from previous TLG file
        break;
      }
    } else {
      break;
    }
  };


  $f = @fopen($dir.$IGCfilename, 'a');
  if ($f === false) {
    http_response_code(500);
    echo "ERROR writing $dir$IGCfilename\n";
    die();
  };


  // I list (B-records)
  $iList = array();
  Vlist_build($iList,'fixAccuracy','FXA',3);			// m (calculated from DOP and fix_quality)
  Vlist_build($iList,'satellites','SIU',2);
  Vlist_build($iList,'speed','GSP',3,3.6);			// m/s -> kph
  Vlist_build($iList,'heading','TRT',3);			// deg
  if ($info['have']['climbrate'])
    Vlist_build($iList,'climbrate','VAR',4,10,true);		// 0.1 m/s
  if ($info['have']['turnrate'])
    Vlist_build($iList,'turnrate','XTU',4,10,true);		// 0.1 deg/s
  if ($info['have']['audio_noise'])
    Vlist_build($iList,'audio_noise','ENL',3);

  // J list (K-records)
  $jList = array();
  if ($info['have']['voltage'])
    Vlist_build($jList,'voltage','XVB',6,1000);			// mV
  if ($info['have']['temp'])
    Vlist_build($jList,'sts_temp','OAT',3,1,true);		// C
  if ($info['have']['hum'])
    Vlist_build($jList,'sts_hum','HUM',3);			// percent

  // M list (N-records)
  $mList = array();
  if ($info['have']['pulse'])
    Vlist_build($mList,'pulse','HRT',3);			// bpm
  if ($info['have']['oxygen'])
    Vlist_build($mList,'oxygen','OXY',3);			// percent


  if (!$append_igc_file) {
    // IGC header
    fwrite($f,'AXOG'.substr($data['file_id'],-6).'-'.$data['file_mac'].IGCEOL);
    fwrite($f,'HFDTEDATE:'.date('ymd',$data['file_time']).','.sprintf("%02d",$file_serial).IGCEOL);
    fwrite($f,'HFPLTPILOTINCHARGE:'.$info['Pilot'].
		(($info['PilotID'] ?? '') !== '' ? ' ('.$info['PilotID'].')' : '' ).IGCEOL); 
    fwrite($f,'HFCM2CREW2:'.$info['Crew'].IGCEOL); 
    fwrite($f,'HFGTYGLIDERTYPE:'.$info['Type'].IGCEOL); 
    fwrite($f,'HFGIDGLIDERID:'.$info['Reg'].IGCEOL); 
    fwrite($f,'HFDTMGPSDATUM:WGS84'.IGCEOL); 
    fwrite($f,'HFRFWFIRMWAREVERSION:'.$info['firmware'].
		(($info['Soft'] ?? '') !== '' ? ','.$info['Soft'] : '').IGCEOL); 
    fwrite($f,'HFRHWHARDWAREVERSION:'.$info['hardware'].
		(($info['Hard'] ?? '') !== '' ? ','.$info['Hard'] : '').IGCEOL); 
    fwrite($f,'HFFTYFRTYPE:XOG,OGN-tracker'.
		(($_SERVER['HTTP_X_OGNTRACKER_PLATFORM'] ?? NULL) ? '/'.($_SERVER['HTTP_X_OGNTRACKER_PLATFORM']) : '') .IGCEOL);

    fwrite($f,'HFGPSRECEIVER:UNK,unknown,999,99999'.IGCEOL); 

    if (isset($info['have']['pressure'])) {
      $_baroSens = ($_SERVER['HTTP_X_OGNTRACKER_BAROSENSOR'] ?? '');
      switch ($_baroSens) {
        case 'MS5607' : { fwrite($f,'HFPRSPRESSALTSENSOR:TE_Connectivity,MS5607,30000'.IGCEOL); break; };
        case 'MS5611' : { fwrite($f,'HFPRSPRESSALTSENSOR:TE_Connectivity,MS5611,30000'.IGCEOL); break; };
        case 'BMP180' : { fwrite($f,'HFPRSPRESSALTSENSOR:Bosch,BMP180,9000'.IGCEOL); break; };
        case 'BMP280' : { fwrite($f,'HFPRSPRESSALTSENSOR:Bosch,BMP280,9000'.IGCEOL); break; };
        case 'BME280' : { fwrite($f,'HFPRSPRESSALTSENSOR:Bosch,BMP/BME280,9000'.IGCEOL); break; };
        default       : { fwrite($f,'HFPRSPRESSALTSENSOR:unknown,unknown,99999'.IGCEOL); break; };
      }
    } else
      fwrite($f,'HFPRSPRESSALTSENSOR:none,none,0'.IGCEOL);
  
    fwrite($f,'HFFRSSECURITYSUSPECT:device has no security'.IGCEOL);
    if ($info['ID'] !== '')
      fwrite($f,'HSCIDCOMPETITIONID:'.$info['ID'].IGCEOL);
    if ($info['Class'] !== '')
      fwrite($f,'HSCCLCOMPETITIONCLASS:'.$info['Class'].IGCEOL);

    fwrite($f,'HOSOFDOWNLOADSOFTWARE:'.$GLOBALS['prog_name'].','.$GLOBALS['version'].','.date('YmdHi').IGCEOL);
    fwrite($f,'HFALGGNNSALTREFERENCE:GEO'.IGCEOL);
    fwrite($f,'HFALPPRESSUREREFERENCE:ISA'.IGCEOL);

    fwrite($f,'LXOGTRACKERMAC:'.$data['file_mac'].IGCEOL);
    fwrite($f,'LXOGTRACKERID:'.$data['file_id'].IGCEOL);
    fwrite($f,'LXOGSOURCEFILE:'.basename($data['filename']).IGCEOL);
    fwrite($f,'LXOGTLGFILETIME:'.$data['file_time'].IGCEOL);

    // write remaining info fields
    foreach (array('Manuf', 'Model', 'SN', 'Task', 'Base', 'ICE') as $_remInfo) {
    if ($info[$_remInfo] ?? NULL)
      fwrite($f,'LXOG'.$_remInfo.':'.$info[$remInfo].IGCEOL);
    };
      

    fwrite($f,Vlist_header($iList, 'I', 35));
    fwrite($f,Vlist_header($jList, 'J', 7));
    fwrite($f,Vlist_header($mList, 'M', 7));
  } else {
    // appending to existing file, make a note
    fwrite($f,'LXOGSOURCEFILE:'.basename($data['filename']).IGCEOL);
    fwrite($f,'LXOGTLGFILETIME:'.$data['file_time'].IGCEOL);
  }


  // MAIN LOOP
  global $last_pos_packet;
  global $last_sts_packet;

  $last_pos_packet = array();
  $last_sts_packet = array();

  foreach ($data['entries'] as $e) {

    //
    // FOREIGN (RADIO-RECEIVED) PACKET

    if (($e['ogn_rx'] ?? NULL) == 1 || $e['h_address'] != $info['tracker_id']) {
      if ($e['report_type'] === -1) {
        $_l = 'EOA1'.sprintf("%06X",$e['h_address']).':C';
        $_l .= igc_coord($e['latitude'],$e['longitude']);
        if (($e['baroaltdiff'] ?? NULL) !== NULL)
          $_l .= sprintf("%05d",$e['altitude'] + $e['baroaltdiff']);
        else
          $_l .= "?????";
        $_l .= sprintf("%05d",$e['altitude']);
      }
      fwrite($f,$_l.IGCEOL);


      // add LXOG line with more data from received packet
      $_l = '';
      switch ($e['report_type']) {
        case -1 : {				// position report
          $_l = sprintf("LXOGEOA1%06X%s:Rly:%d,ACTyp:%d,FixQ:%d,DOP:%d,FixMod:%d,Spd:%.1f,TrnRte:%.1f,Hdg:%d,Clmb:%.1f",
	              $e['h_address'], ($e['h_emergency'] ? '-EMG,':''), $e['h_relay'], $e['acfttype'], $e['fix_quality'], $e['dop'], $e['fixmode'],
	            	$e['speed'], $e['turnrate'], $e['heading'], $e['climbrate']);
          break;
        };
        case 0 : {				// status report
          $_l = sprintf("LXOGEOA1%06X%s:Rly:%d,SatNR:%d,Anois:%d,Rnois:%.1f,Tmp:%.1f,Hum:%d,Pres:%.1f,Sats:%d,Fw:%d,Hw:%d,TxPwr:%d,V:%.2f",
		            $e['h_address'], ($e['h_emergency'] ? '-EMG':''), $e['h_relay'], $e['sat_SNR'], $e['audio_noise'] ?? -1,
            		$e['radio_noise'], $e['sts_temp'] ?? -99, $e['sts_hum'] ?? -1, $e['sts_pressure'] ?? -1, $e['satellites'],
	            	$e['firmware'], $e['hardware'], $e['tx_power'], $e['voltage'] ?? -1);
          break;
        };
        case 1 : {				// info report
           $_l = sprintf("LXOGEOA1%06X:", $e['h_address']);
          foreach ($e['info'] as $_in => $_iv)
            $_l .= $_in.':'.$_iv.',';
          $_l = substr($_l,0,-1);
          break;
        };

        default : {
          $_l = '';
          break;
        };
      };
      if ($_l !== '')
        fwrite($f,$_l.IGCEOL);
      
      continue;
    };


    //
    // PROCESS NORMAL (OWN) PACKETS

    if (isset($e['dop'])) {
      $e['fixAccuracy'] = min(63,($e['dop'] * 2 + 5) / 10);
    };

    if (isset($e['audio_noise']))
      $e['audio_noise'] += 10;

    switch ($e['report_type']) {
      case -1 : {				// position report
        $last_pos_packet = $e;
        $_l = 'B'.date('His',$e['unixtime']);
        $_l .= igc_coord($e['latitude'],$e['longitude']);
        $_l .= ($e['fix_quality'] >= '1' ? 'A' : 'V');
        if (($e['baroaltdiff'] ?? NULL) !== NULL)
          $_l .= sprintf("%05d",$e['altitude'] + $e['baroaltdiff']);
        else
          $_l .= "?????";
        $_l .= sprintf("%05d",$e['altitude']);
        $_l .= Vlist_fill($iList, $e);
 
        fwrite($f,$_l.IGCEOL);
        break;
      };

      case 0 : {				// status packet
        $last_sts_packet = $e;

        if (count($jList) > 0) {
          $_l = 'K'.date('His',$e['unixtime']);
          $_l .= Vlist_fill($jList, $e);
          fwrite($f,$_l.IGCEOL);
        };

        if (count($mList) > 0) {
          $_l = 'N'.date('His',$e['unixtime']);
          $_l .= Vlist_fill($mList, $e);
          fwrite($f,$_l.IGCEOL);
        };

        break;
      };

      case 1 : {				// info packet
        // values from info packets are written once in the IGC file header
        break;
      };
    };
  };

  fclose($f);
  return true;
};


function check_append($file,$data) {

  // This won't work if flight spans across UTC midnight - sorry Australia!
  if (!isset($data['file_time']))							// that shouldn't happen
    return false;

  $f = @fopen($file,'r');
  if ($f === false)
    return false;

  while (!feof($f)) {
    $l = trim(fgets($f,256));
    if (strlen($l) == 0)
      continue;


    if (substr($l,0,5) == 'HFDTE') {
      $ref_date = explode(':',trim($l));
      $ref_date = $ref_date[1] ?? '';
      $ref_date = explode(',',$ref_date);
      $ref_date = $ref_date[0];

      if ($ref_date != date('ymd',$data['file_time']))					// different dates
        return false;									// that shouldn't happen
    }

    if (substr($l,0,16) == 'LXOGTLGFILETIME:' && !isset($last_file_time)) {
      $last_file_time = explode(':',($l));
      $last_file_time = (int) ($last_file_time[1] ?? NULL);
    };


    if ($l[0] != 'B')
      continue;

    if (!isset($first_B))
      $first_B = substr($l,1,6);

    $last_B = substr($l,1,6);
  };

  fclose($f);

  if (($last_B ?? NULL) == '')					// no B records found!
    return false;

  $time_ref=date('His',$data['file_time']);
  $time_ref=(int) substr($time_ref,0,2) * 3600 + (int) substr($time_ref,2,2) * 60 + (int) substr($time_ref,4,2);

  $time_start=(int) substr($first_B,0,2) * 3600 + (int) substr($first_B,2,2) * 60 + (int) substr($first_B,4,2);
  $time_end=(int) substr($last_B,0,2) * 3600 + (int) substr($last_B,2,2) * 60 + (int) substr($last_B,4,2);

  if (isset($data['info']['first_pos_time']) && isset($data['info']['last_pos_time'])) {
    $f_pos_B = date('His',$data['info']['first_pos_time'] ?? 0);
    $l_pos_B = date('His',$data['info']['last_pos_time'] ?? 0);

    $ref_start=(int) substr($f_pos_B,0,2) * 3600 + (int) substr($f_pos_B,2,2) * 60 + (int) substr($f_pos_B,4,2);
    $ref_end  =(int) substr($l_pos_B,0,2) * 3600 + (int) substr($l_pos_B,2,2) * 60 + (int) substr($l_pos_B,4,2);

    if (abs($ref_start - $time_start) <= 30 && abs($ref_end - $time_end) <= 30 )		// same file, overwrite
      return -1;
  } else {							// weird case: no B lines in file
    if ((int) $last_file_time === (int) $data['file_time'])	// same file, overwrite
      return -1;
  };

  if ($time_ref + 30 < $time_end)			// files overlapping a lot, weird!
    return false;

  if ($time_ref - $time_end < 180)			// continuation, append to this IGC file
    return true;
  
  return false;						// different flights, create new IGC file
};


function Vlist_build( & $list,$name,$tlc,$len,$factor = 1, $allowNeg = false) {
  if (count($list) > 0) {
    $last_c = end($list);
    $last_c = $last_c['c_end'];
  } else {
    $last_c = 0;
  };

  $list[$name] = array(
	'tlc' => $tlc,
	'c_start' => $last_c + 1,
	'c_end' => $last_c + $len,
	'len' => $len,
	'allowNeg' => $allowNeg,
	'factor' => $factor,
  );

  $list[$name]['strF'] = "%0{$len}d";

  if ($allowNeg) {
    $list[$name]['min'] = 1 - 10**($len-1);
    $list[$name]['max'] = 10**($len) - 1;
  } else {
    $list[$name]['min'] = 0;
    $list[$name]['max'] = 10**($len) - 1;
  };

  return true;
};


function Vlist_header($list, $lineType, $offset = 0) {
  if (count($list) == 0)
    return '';

  $res = $lineType.sprintf("%02d",count($list));
  foreach ($list as $m)
    $res .= sprintf("%02d%02d%3s",$m['c_start']+$offset,$m['c_end']+$offset,$m['tlc']);

  return $res.IGCEOL;
};


function Vlist_fill($list, $e) {
  global $last_pos_packet;
  global $last_sts_packet;

  if (count($list) == 0)
    return '';

  $res = ''; 

  foreach ($list as $n => $m) {
    $v = $e[$n] ?? $last_sts_packet[$n] ?? $last_pos_packet[$n] ?? NULL;

    if ($v === NULL) {
      $res .= str_pad('',$m['len'],'?');
    } else {
      if ($m['factor'] !== NULL && $m['factor'] !== 1)
        $v *= $m['factor'];
      $v = max(min($m['max'],$v),$m['min']);
      $res .= substr(sprintf($m['strF'],$v),0,$m['len']);		// substr is just in case
    };
  };

  return $res;
};


function igc_coord($lat,$lon) {
  $lat_d = sprintf("%02d",floor(abs($lat)));
  $lat_m = sprintf("%06.3f",(abs($lat)-$lat_d)*60);
  $lat_m = str_replace('.','',$lat_m);
  $lat_sign = ($lat < 0) ? 'S' : 'N';

  $lon_d = sprintf("%03d",floor(abs($lon)));
  $lon_m = sprintf("%06.3f",(abs($lon)-$lon_d)*60);
  $lon_m = str_replace('.','',$lon_m);
  $lon_sign = ($lon < 0) ? 'W' : 'E';

  return "$lat_d$lat_m$lat_sign$lon_d$lon_m$lon_sign";
};


function trackerSubDir($filename, $acftid) {
  if (strlen($filename) < 6)
    return '';

  list($file_mac,$file_id,$file_time) = decode_filename($filename);
  if ($file_id == NULL)
    $file_id = $acftid;
  if ($file_id === NULL || strlen($file_id) < 6)
    return '';

  if ($df = @fopen(TRACKERS_DICTIONARY,'r')) {
    while (!feof($df)) {
      $la = explode(',',trim(fgets($df,128)));
      if (count($la) < 2)
        continue;
      if (strlen($la[0]) >= 6 && strtoupper($la[0]) === strtoupper($file_id)) {
        $name = $la[1] ?? NULL;

        fclose($df);
        if (strlen($name) > 0 && strpos($name,'/') === false && $name[0] !== '.' )	// no subdirectories allowed
          return $name.'/';						// first match counts
        else 
          return $file_id.'/';
      } else {
        continue;
      }
    }
    fclose($df);
  };
  
  return $file_id.'/';
}


?>