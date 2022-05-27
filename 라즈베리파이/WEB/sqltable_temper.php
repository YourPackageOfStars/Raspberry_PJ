<!DOCTYPE html>
<html>
<head>
	<meta charset = "UTF-8">
	<meta http-equiv= "refresh" content ="30">
	<style type = "text/css">
		.spec{
			text-align:center;
		}
		.con{
			text-align:left;
		}
		</style>
</head>

<body>
	<h1 align = "center">My Database</h1>
	<div class ="spec">
		# <b>The sensor value desciption</b><br>
		# 1 ~ 99 TEMPERATURE <br>
	</div>

	<table border = '1' style = "width = 30%" align = "center">
	<tr align = "center">
		<th>TIME</th>
		<th>TEMPERATURE</th>
	</tr>
	<?php
		$conn = mysqli_connect("localhost","root","kcci");
		mysqli_select_db($conn, "RaspPJ");
		$result = mysqli_query($conn, "select * from temper");

		while($row = mysqli_fetch_array($result)){
		echo "<tr align = center>";
		echo '<td>'.$row['TIME'].'</td>';
		echo '<td>'.$row['TEMPERATURE'].'</td>';
		echo "<tr>";
		mysqli_close($con);
		}
	?>
	</table></body></htmle>
