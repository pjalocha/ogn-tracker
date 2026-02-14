<?
function parse_tlg($filename) {
  list($file_mac,$file_id,$file_time) = decode_filename($filename);
  $unixtime = $file_time;

  $fi = @fopen($filename,'r');
  if ($fi == false)
    return NULL;

  $res = array(
    'filename' => $filename,
    'file_mac' => $file_mac,
    'file_id' => $file_id,
    'file_time' => $file_time,
    'entries' => array(),
  );

  while (!feof($fi)) {
    $p=fread($fi,24);

    if (strlen($p) == 0)
      break;

    $ogn = unpack_ogn1($p);

    if (isset($ogn['time_s'])) {
      $time16 = (int) getTime16($unixtime,$ogn['ogn_time16']);
      $unixtime = restoreTime($time16,$ogn['time_s']);
    };
    $ogn['unixtime'] = $unixtime;
    $ogn['_time'] = date('H:i:s',$unixtime);
    $res['entries'][] = $ogn;
  };

  fclose($fi);

  return ($res);
};

function unpack_ogn1($p) {
  $o = array();
  $o['ogn_time16'] = (int) (reset(unpack('S', $p, 20))) << 4;
  $o['ogn_SNR'] = (int) (reset(unpack('C', $p, 22)) && 0b00111111);
  $o['ogn_prot'] = (int) (reset(unpack('C', $p, 22)) && 0b01000000) >> 6;
  $o['ogn_rx'] = (int) (reset(unpack('C', $p, 22)) && 0b10000000) >> 7;
  $o['ogn_checksum'] = (int) (reset(unpack('C', $p, 23)));

  $o['h_address'] = (int) (reset(unpack('V', $p, 0)) & 0x00ffffff);
  $o['h_addr_type'] = (int) (reset(unpack('C', $p, 3)) & 0b00000011);
  $o['h_nonpos'] = (int) (reset(unpack('C', $p, 3)) & 0b00000100) >> 2;
  $o['h_parity'] = (int) (reset(unpack('C', $p, 3)) & 0b00001000) >> 3;
  $o['h_relay'] = (int) (reset(unpack('C', $p, 3)) & 0b00110000) >> 4;
  $o['h_encrypted'] = (int) (reset(unpack('C', $p, 3)) & 0b01000000) >> 6;
  $o['h_emergency'] = (int) (reset(unpack('C', $p, 3)) & 0b10000000) >> 7;

  $o['report_type_txt'] = 'unknown';
  if ($o['h_encrypted']) {
    $o['report_type_txt'] = 'custom/encrypted';
    return ($o);
  };


  if ($o['h_nonpos'] == 0) {					// position report
    $o['report_type'] = -1;					// there's no number for position packet
    $o['report_type_txt'] = 'position';
    $o['latitude'] = (float) ((reset(unpack('V', $p, 4)) & 0x00ffffff) * 8 + 4) * (0.0001/60);
    $o['time_s'] = (int) (reset(unpack('C', $p, 7)) & 0b00111111);
    $o['fix_quality'] = (int) (reset(unpack('C', $p, 7)) & 0b11000000 >> 6);

    $o['longitude'] = (float) ((reset(unpack('l', $p, 8)) & 0x00ffffff) * 16 + 8) * (0.0001/60);
    $o['dop'] = (int) decodeVRuns(reset(unpack('C', $p, 11)) & 0b00111111, 4) + 10;	// GPS dillution of precision
    $_baroMSB = (int) (reset(unpack('C', $p, 11)) & 0b01000000) >> 6;
    $o['fixmode'] = (int) ((reset(unpack('C', $p, 11)) & 0b10000000) >> 7);		// 0=2d, 1=3d

    $o['altitude'] = (int) decodeVRuns(reset(unpack('v', $p, 12)) & 0x3fff, 12);
    $o['speed'] = (float) decodeVRuns((reset(unpack('v', $p, 13)) & 0xffffc0) >> 6, 8) / 10;	// m/s

    $o['turnrate'] = reset(unpack('C', $p, 15));
    if ($o['turnrate'] == 0x80)						// 0x80 means no turnrate
      $o['turnrate'] = NULL;
    else
      $o['turnrate'] = (float) (decodeVRsig($o['turnrate'], 5)) / 10;			// deg/s

    $o['heading'] = (float) ((reset(unpack('v', $p, 16)) & 0x03ff) * 3600 + 512) / 10240;

    $o['climbrate'] = (reset(unpack('v', $p, 17)) & 0x07fc) >> 2;
    if ($o['climbrate'] == 0x100)					// 0x100 means no climbrate
      $o['climbrate'] = NULL;
    else
      $o['climbrate'] = (float) (decodeVRsig($o['climbrate'], 6)) / 10;			// m/s

    $o['stealth'] = (int) ((reset(unpack('C', $p, 18)) & 0b00001000) >> 3);
    $o['acfttype'] = (int) ((reset(unpack('C', $p, 18)) & 0b11110000) >> 4);

    $o['baroaltdiff'] = (int) reset(unpack('C', $p, 19));				// m
    if ($o['baroaltdiff'] == 0 && $_baroMSB == 0)		// invalid / no baro / out of range
      $o['baroaltdiff'] = NULL;
    elseif ($_baroMSB == 0)					// negative value
      $o['baroaltdiff'] = $o['baroaltdiff']-256;



  } else {						// non-pos report, content depends on report_type
    $o['report_type'] = (int) ((reset(unpack('C', $p, 18)) & 0xf0) >> 4);

    switch ($o['report_type']) {
      case '0' : {				// status report
        $o['report_type_txt'] = 'status';

        $o['sts_unixtime'] = (int) (reset(unpack('L', $p, 4)));

        $o['pulse'] = (int) (reset(unpack('C', $p, 4)));
        $o['oxygen'] = (int) (reset(unpack('C', $p, 5)) & 0b01111111);
        $o['sat_SNR'] = (int) ((reset(unpack('v', $p, 5)) & 0x0f80) >> 7) + 8;
        $o['rx_rate'] = (int) (reset(unpack('C', $p, 6)) & 0xf0) >> 4;
        $o['time_s'] = (int) (reset(unpack('C', $p, 7)) & 0b00111111);
        $o['fix_quality'] = (int) (reset(unpack('C', $p, 7)) & 0b11000000) >> 6;


        $o['audio_noise'] = (int) (reset(unpack('C', $p, 8)));					// dB
        $o['radio_noise'] = (float) (reset(unpack('C', $p, 9))) * -0.5;				// dBm

        $o['sts_temp'] = reset(unpack('C', $p, 10));
        if ($o['sts_temp'] == 0x80)				// 0x80 means no temperature
          $o['sts_temp'] = NULL;
        else
          $o['sts_temp'] = (float) (decodeVRsig($o['sts_temp'], 5)+200) / 10;			// C

        $o['sts_hum'] = reset(unpack('C', $p, 11));
        if ($o['sts_hum'] == 0x80)				// 0x80 means no humidity
          $o['sts_hum'] = NULL;
        else
          $o['sts_hum'] = (float) (decodeVRsig($o['sts_hum'], 5)+520) / 10;			// %


        $o['sts_altitude'] = (int) decodeVRuns((reset(unpack('v', $p, 12)) & 0x3fff), 12);	// m
        $o['sts_pressure'] = (float) ((reset(unpack('V', $p, 12)) & 0x0fffc000) >> 14) * 0.08;
        $o['satellites'] = (int) (reset(unpack('C', $p, 15)) & 0xf0) >> 4;			// dBm


        $o['firmware'] = (int) (reset(unpack('C', $p, 16)));
        $o['hardware'] = (int) (reset(unpack('C', $p, 17)));
        $o['tx_power'] = (int) (reset(unpack('C', $p, 18)) & 0x0f) + 4;
        $o['voltage'] = (float) decodeVRuns(reset(unpack('C', $p, 19)), 6) / 64 ;		// V

        break;
      };

      case '1' : {				// info packet
        $o['report_type_txt'] = 'info';
        $_data_chars = (int) ((reset(unpack('C', $p, 18)) & 0b00001111));
        $_data = substr($p,4,14);

        $_data = strrev($_data);
        $bits = '';
        for ($i = 0; $i < 14; $i++) {
          $bits .= str_pad(decbin(ord($_data[$i])), 8, '0', STR_PAD_LEFT);
        }

        $info = '';
        for ($i = 0; $i < strlen($bits); $i += 7) {
          if ($i + 7 <= strlen($bits))
            $info .= chr(bindec(substr($bits,$i,7)));
        }

        $info = strrev($info);

        $infoParDict = array ("Pilot", "Manuf", "Model", "Type", "SN", "Reg", "ID", "Class",
				"Task" , "Base" , "ICE"  , "PilotID", "Hard", "Soft", "Crew" );

        $_i = '';
        for ($i=0;$i<=$_data_chars;$i++) {
          if (ord($info[$i]) < 16) {		// separator
            if (strlen($_i))
              $o['info'][$infoParDict[ord($info[$i])]] = $_i;
            $_i = '';
          } else {
            $_i .= $info[$i];
          }
        };

        break;
      };

      case '2' : {				// wind/thermal report
						// not supported yet
        $o['report_type_txt'] = 'wind-therm';
        break;
      };

      case '3' : {				// meteo raport
						// not supported yet
        $o['report_type_txt'] = 'meteo';
        break;
      };

      case '15' : {				// custom / manufacturer specific
						// not supported yet
        $o['report_type_txt'] = 'manuf';
        break;
      };
    };
  }

  return $o;
}


function decodeVRuns($V,$bits) {			// encoded value, raw value bits (rest are range bits)
  // unsigned Variable-Range value decode

  $Thres = 1 << $bits;
  $range = $V >> $bits;

  // extract raw value
  $val = $V & ($Thres - 1);

  // apply variable range correction
  if ($range > 0)
    $val = (2**$range-1) * $Thres + (1 << ($range-1)) + ($val<<$range);

  return $val;
};


function decodeVRsig($V,$bits, $Rbits = 2) {		// encoded value, raw value bits, range bits
  // signed Variable-Range value decode

  // extract sign bit and remove it from range bits
  $signmask = 1 << ($bits+$Rbits);
  $sign = $V & $signmask;
  $V = $V & ($signmask - 1);

  if ($sign)		// 1 means negative
    return -decodeVRuns($V,$bits);
  else
    return decodeVRuns($V,$bits);
};


function getTime16($refTime, $time16) {			// reference (approximate) time, truncated time
  $refTime >>= 4;
  $time16 >>= 4;
 
  $diff = ($time16 - $refTime) % 65536;
  if ($diff < 0)
    $diff += 65536;
  if ($diff >= 32768)
    $diff -= 65536;


  $refTime += $diff;
  $refTime <<= 4;
  $refTime += 15;

  return $refTime;
}


function restoreTime($time16, $time_s) {		// $time16 is unix time with last 4 bits = 0000
  $time16_s = (int) date('s',$time16);

  $b = $time16 % 60;
  $d = ($time_s - $b + 60) % 60;

  if ($d > 15) {
    $d -= 60;
  }

  return $time16 + $d;
};



function decode_filename($filename) {
  $filename = basename($filename);
  if ((strtoupper(substr($filename,-4)) == '.TLG') || (uppercase(substr($filename,-4)) == '.TLA'))
    $filename = substr($filename,0,-4);

  $a= explode('_',$filename);

  if (count($a) >= 3) {
    $mac = $a[0];
    $id = $a[1];
    $time = hexdec($a[2]);
  };

  return array($mac ?? NULL, $id ?? NULL, $time ?? NULL);
};

?>