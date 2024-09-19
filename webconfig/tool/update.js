//
// update.js
//

//
// Create a centered prompt string
//
function makeCenteredPromptString(thePrompt){
	return '<p style="font-size:12pt;line-height:18pt;font-family:helvetica;text-align:center">'+thePrompt+'</p>';
}


//
// Unregister the service worker and force an update
//

function UpdateToLatestVersion(){

	ForceUpdate(callback);

	function callback(restartRequested){

		if (restartRequested){

			//debugger;

			setTimeout(function(){

				var thePrompt = "All changes applied. Click OK to restart the tool.";
				
				// Center the string in the prompt
				thePrompt = makeCenteredPromptString(thePrompt);

				DayPilot.Modal.alert(thePrompt,{ theme: "modal_flat", top: 320, scrollWithPage: false }).then(function(){

					window.location.reload();

				});

			},1000);
		}
	}
}


function ForceUpdate(callback){

	var thePrompt = "This will force the version of the tool stored in<br/>your browser to be updated after a restart.<br/><br/>After the restart, you may need to refresh<br/>the page one more time to use the update.<br/><br/>Are you sure?";

	// Center the string in the prompt
	thePrompt = makeCenteredPromptString(thePrompt);

	DayPilot.Modal.confirm(thePrompt,{ top:262, theme: "modal_flat", scrollWithPage: false }).then(function(args){

		if (!args.canceled){

			console.log("Tool update requested");
			
			// Give some time to show the spinner
			setTimeout(function(){
				if ('serviceWorker' in navigator) {
				  navigator.serviceWorker.getRegistrations().then(function(registrations) {
				    let unregisterPromises = [];
				    
				    for (let registration of registrations) {
				      if (registration.scope.indexOf("warbl") != -1){
				      	console.log("Unregistering service worker with scope: "+registration.scope);
				      	unregisterPromises.push(registration.unregister());
				      }
				    }
				    
				    Promise.all(unregisterPromises).then(function() {
				      console.log('All service workers unregistered');
				      // Call your callback function here
				      callback(true);
				    }).catch(function(error) {
				      console.error('Error unregistering service workers:', error);
				      // Optionally call the callback function in case of error
				      callback(true);
				    });
				  }).catch(function(error) {
				    console.error('Error getting service worker registrations:', error);
				    // Optionally call the callback function in case of error
				    callback(true);
				  });
				} 
				else {
				 
					console.warn('Service workers are not supported in this browser.');

					// Optionally call the callback function if service workers are not supported
					callback(true);
				}

			},100);


		}
		else{

			//console.log("Cancelled database delete");

			callback(false);

		}
	});
}


