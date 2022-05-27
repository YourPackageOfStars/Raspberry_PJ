<?php
	$conn = mysqli_connect("localhost","root","kcci");
	mysqli_set_charset($conn, "utf-8");
	mysqli_select_db($conn, "RaspPJ");
	$result=mysqli_query($conn, "select TIME, TEMPERATURE from temper");

	$data = array(array('온도','TEMPERATURE'));
	if($result){
		while($row=mysqli_fetch_array($result))
		{
			array_push($data, array($row[0], intval($row[1])));
			herder("Refresh:3");
		}
	}
	$options = array(
	    'title' => '온도(단위:섭씨)',
		'width' => 1000, 'height' => 500
	);
?>

<script src="//www.google.com/jsapi"></script>
<script>
    let data = <?= json_encode($data) ?>;
    let options = <?= json_encode($options) ?>;
    google.load('visualization', '1.0', {'packages':['corechart']});
    google.setOnLoadCallback(function() {
        let chart = new google.visualization.LineChart(document.querySelector('#chart_div'));
        chart.draw(google.visualization.arrayToDataTable(data), options);
    });
</script>
<div id="chart_div"></div>
