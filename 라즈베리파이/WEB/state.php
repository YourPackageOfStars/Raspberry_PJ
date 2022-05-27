<!DOCTYPE html>
<html lang="ok">
	<head>
		<meta http-equiv="Content_Type"
			content="text/html;charset=UTF-8"/>
		<title>sample page</title>
	</head>
	<body>
		<herder>
		<h1>1</h1>
		<p>
		<?php
			readfile("/home/pi/khm/raspberryPJ/state.txt");
		 	header("Refresh:2");
		?></p>
		</herder>
	</body>
</html>
