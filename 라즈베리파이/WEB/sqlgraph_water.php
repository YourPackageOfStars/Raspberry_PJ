
<?php
    $conn = mysqli_connect("localhost","root","kcci");
    mysqli_set_charset($conn, "utf-8");
    mysqli_select_db($conn, "RaspPJ");
    $result=mysqli_query($conn, "select WATERLEVEL from water");

    $data = array(array('물높이','WATERLEVEL'));
    if($result){
        while($row=mysqli_fetch_array($result))
        {
            array_push($data, array($row[0], intval($row[0])));
			header("Refresh:2");
        }
    }
    $options = array(
        'title' => '물높이',
        'width' => 700, 'height' => 700
    );
?>

<script src="//www.google.com/jsapi"></script>
<script>
    let data = <?= json_encode($data) ?>;
    let options = <?= json_encode($options) ?>;
    google.load('visualization', '1.0', {'packages':['corechart']});
    google.setOnLoadCallback(function() {
        let chart = new google.visualization.ColumnChart(document.querySelector('#chart_div'));
        chart.draw(google.visualization.arrayToDataTable(data), options);
    });
</script>
<div id="chart_div"></div>
