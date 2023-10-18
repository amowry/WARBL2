			/*

			if (version > 1.4){ //add new items that should only be visible with newer software versions
				
				//add standard uilleann fingering option (no accidentals) if version is greater than 1.4
				for (i = 0; i < document.getElementById("fingeringSelect0").length; ++i){
    				if (document.getElementById("fingeringSelect0").options[i].value == "10"){
 						var a = 1;    
				}}

  				if (a != 1){
					var x = document.getElementById("fingeringSelect0"); 		
  					var option = document.createElement("option");
  					option.text = "Uilleann pipes, standard";
  					option.value = 10;
  					x.add(option);
		
					var y = document.getElementById("fingeringSelect1");
					var option = document.createElement("option");
					option.text = "Uilleann pipes, standard";
  					option.value = 10;
					y.add(option);
		
					var z = document.getElementById("fingeringSelect2");
					var option = document.createElement("option");
					option.text = "Uilleann pipes, standard";
  					option.value = 10;
					z.add(option);

				}
	
							
				//add semitone shift option to button configuration and velocity send to expression panel if the WARBL software version is greater than 1.4				
				var a = 0;
				for (i = 0; i < document.getElementById("row0").length; ++i){
    				if (document.getElementById("row0").options[i].value == "10"){ //check to see if we've already added them
 					a = 1;    
					}}
					
				if (a != 1){
					for (var i=0; i< 8; i++) {	
					
						var x = document.getElementById("row" + i); 		
  						var option = document.createElement("option");
  						option.text = "Semitone shift up";
  						option.value = 10;
  						x.add(option);
					
						var y = document.getElementById("row" + i); 		
  						var option = document.createElement("option");
  						option.text = "Semitone shift down";
  						option.value = 11;
  						y.add(option);
					}
				}
				
				document.getElementById("switch10").style.visibility = "visible";
				document.getElementById("velocityLabel").style.visibility = "visible";
				document.getElementById("switch7").style.top = "140px";
				document.getElementById("sendPressureLabel").style.top = "143px";
				
				
				
			}
			
			*/