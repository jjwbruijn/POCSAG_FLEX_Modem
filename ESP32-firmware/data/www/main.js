$.fn.htmlTo = function(elem) {
	return this.each(function() {
		$(elem).html($(this).html());
	});
}

let url = 'ws://10.0.15.178/ws'; //localhost?mischien ergens vandaan halen?
let socket = new WebSocket(url);

socket.onmessage = function(event) {
	let incomingMessage = event.data;
	showMessage(incomingMessage);
};

socket.onclose = event => console.log(`Closed ${event.code}`);

function showMessage(message) {
	try {
		let data = JSON.parse(message);
		if(data.frames) {
			var blocks = [];
			$.each( data.frames, function(key, val) {

				var freq = "--";	
				if(val.freq !== 0) {
					var freq = val.freq / 1000000 + "MHz";
				}
				if(val.txType) {
					var type = val.txType;
					var c = 'txt';
				}
				if(val.rxType) {
					var type = val.rxType;
					var c = 'rxt';
				}
				blocks.push( "<li><div class=\"b " + c + "\"><div>" + val.frame + "</div><div>" + type + "</div><div>" + freq + "</div></li>" );
			});

		} else {

			if(data.frame) {
				//data.frame, data.cycle, data.type
				newDate = new Date(data.time * 1000);
				var date = newDate.toLocaleString();
				var t = date.split(" ");
				var time = t[1];
				if(data.messages !== 0) {
					var items =  [];
					$.each(data.messages, function(k, v) {
						if(v.msg.length) {
							items.push("<div class=\"date\">" + time + "</div><div class=\"message\">" + v.msg + "</div>");
							$(".pending").remove();
							$( "<li/>", {
								"class": "new",
								html: items.join("")
							}).prependTo( "ul.messages" );
						}
					});

				}
			}
		}

	} catch(err) {


	}

	if(blocks) {
		$( "<ul/>", {
			"class": "",
			html: blocks.join("")
		}).htmlTo( ".blocks ul" );
	}

}
