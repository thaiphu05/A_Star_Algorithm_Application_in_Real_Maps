function getGraph() {
    let location = document.getElementById("location").value;
    try {
        if (location.trim() === '') {
            throw new Error("Please enter a location");
        }
        $("#graphImage").attr("src", "").hide();
        $(".map-container").append('<div id="loading" class="alert alert-info">Loading...</div>');
        
        $.ajax({
            url: "/get_graph",
            type: "POST",
            contentType: "application/json",
            data: JSON.stringify({location: location}),
            success: function(response) {
                $("#loading").remove();
                $("#graphImage").attr("src", response.image_url).show();
            },
            error: function(xhr, status, error) {
                $("#loading").remove();
                alert("Error loading map: " + error);
            }
        });
    } catch (error) {
        alert(error);
        return;
    }
}
